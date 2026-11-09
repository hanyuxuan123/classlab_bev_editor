#!/usr/bin/env python3
"""Local review UI for Classlab boxes, road lines and pedestrian crossings.

Run: python server.py /path/to/clip
Reviews default to /path/to/clip/reviews; original sensor and label files are preserved.
"""

import argparse
import bisect
import csv
import json
import math
import mmap
import os
import uuid
from functools import lru_cache
from pathlib import Path

import yaml
from flask import Flask, abort, jsonify, request, send_file, send_from_directory

from geometry import (apply_boundary_policy, box_corners, frame_offsets,
                      pcd_bev_points, project, read_road_frame, rotate_quaternion)


SOURCE_CLASS = {1: 'pedestrian', 2: 'bicycle', 3: 'car', 4: 'truck',
                5: 'bus', 6: 'truck', 7: 'construction_vehicle',
                8: 'ignore', 9: 'ignore', 10: 'ignore', 11: 'traffic_cone',
                12: 'ignore', 13: 'truck', 14: 'ignore'}
REVIEW_TYPES = {2, 4, 8, 12, 13}
MODEL_CLASSES = ('car', 'truck', 'construction_vehicle', 'bus', 'trailer',
                 'barrier', 'motorcycle', 'bicycle', 'pedestrian', 'traffic_cone', 'ignore')
MAP_CLASSES = ('divider', 'boundary', 'ped_crossing', 'ignore')
EDIT_MAP_CLASSES = ('divider', 'boundary', 'ped_crossing', 'boundary_candidate', 'ignore')
CAMERA_ORDER = ('haomibo_camera', 'front_left_camera', 'right_front_camera',
                'left_camera', 'right_camera', 'left_back_camera', 'back_right_camera')


def finite(value, default=0.0):
    try:
        result = float(value)
        return result if math.isfinite(result) else default
    except (ValueError, TypeError):
        return default


def finite_numbers(values, count):
    return (isinstance(values, list) and len(values) == count
            and all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in values))


def unique_box_ids(boxes):
    """GT.csv can repeat its row id within one frame; UI ids must be unique."""
    used = set()
    for box in boxes:
        original = box['id']
        candidate = original
        suffix = 2
        while candidate in used:
            candidate = '{}#{}'.format(original, suffix)
            suffix += 1
        box['id'] = candidate
        used.add(candidate)


class Clip:
    def __init__(self, root, edits_dir=None, boundary_offset_m=0.5,
                 boundary_policy='semantic', regenerate_auto_boundaries=False):
        if not math.isfinite(boundary_offset_m) or not 0 <= boundary_offset_m <= 3:
            raise ValueError('boundary offset must be between 0 and 3 m')
        if boundary_policy not in ('semantic', 'outer-offset'):
            raise ValueError('unknown boundary policy: {}'.format(boundary_policy))
        self.root = Path(root).resolve()
        self.result = self.root / 'result'
        self.edits = (Path(edits_dir).resolve() / self.root.name
                      if edits_dir is not None else self.root / 'reviews')
        self.edits.mkdir(parents=True, exist_ok=True)
        self.boundary_offset_m = boundary_offset_m
        self.boundary_policy = boundary_policy
        self.regenerate_auto_boundaries = regenerate_auto_boundaries
        self.rules_file = self.edits / 'class_rules.json'
        self.deleted_tracks_file = self.edits / 'deleted_tracks.json'
        self.crossings_file = self.edits / 'crossings_global.json'
        self.boundaries_file = self.edits / 'boundaries_global.json'  # legacy
        self.map_lines_file = self.edits / 'map_lines_global.json'
        self.frames = {}
        self.timestamps = {}
        with (self.result / 'GT.csv').open(newline='') as handle:
            for row in csv.DictReader(handle):
                frame = int(row['frame_num'])
                self.frames.setdefault(frame, []).append(row)
                self.timestamps[frame] = finite(row['stamp_sec'])
        self.frame_ids = sorted(self.frames)
        self.frame_index = {frame: index for index, frame in enumerate(self.frame_ids)}
        self.poses = {}
        with (self.result / 'mapping' / 'mapping_pose_quaterniond.txt').open() as handle:
            for row in csv.DictReader(handle, delimiter=' ', skipinitialspace=True):
                if row.get('lidar_frame') is None:
                    continue
                self.poses[int(row['lidar_frame'])] = (
                    [float(row[a]) for a in 'xyz'],
                    [float(row['orientation_'+a]) for a in 'xyzw'])
        self.road_handle = (self.result / 'road.bin').open('rb')
        self.road_data = mmap.mmap(self.road_handle.fileno(), 0, access=mmap.ACCESS_READ)
        self.road_offsets = frame_offsets(self.road_data)
        if len(self.road_offsets) != len(self.frame_ids):
            raise ValueError('road.bin has {} frames, GT.csv has {}'.format(
                len(self.road_offsets), len(self.frame_ids)))
        self.cameras = self._cameras()
        self.synced = {}
        sync_path = self.root / 'raw' / 'sync_time_list.txt'
        if sync_path.exists():
            with sync_path.open() as handle:
                channels = handle.readline().lstrip('#').split()
                lidar_index = channels.index('/middle')
                for text in handle:
                    values = text.split()
                    if len(values) != len(channels):
                        continue
                    lidar_stamp = int(values[lidar_index])
                    self.synced[lidar_stamp] = {
                        name: int(values[channels.index('/' + name)])
                        for name in self.cameras if '/' + name in channels}
        self.sync_stamps = sorted(self.synced)
        self.images = {}
        for name in self.cameras:
            directory = self.root / 'raw' / 'image' / name
            stamps = sorted(int(p.stem) for p in directory.glob('*.jpeg'))
            self.images[name] = stamps

    def _cameras(self):
        config = yaml.safe_load((self.root / 'car__2024-12-12.yaml').read_text())
        result = {}
        for camera in config['sensors']['camera']:
            name = camera['topic'].strip('/').split('/')[0]
            cal = camera['calibration']
            result[name] = {'ext': cal['CameraExt'], 'K': cal['CameraIntMat'],
                            'D': cal['DistCoeff'], 'size': cal['ImageSize']}
        return {name: result[name] for name in CAMERA_ORDER if name in result}

    def _image(self, frame, name):
        target = round(self.timestamps[frame] * 1e9)
        if self.sync_stamps:
            idx = bisect.bisect_left(self.sync_stamps, target)
            nearby = self.sync_stamps[max(0, idx - 1):min(len(self.sync_stamps), idx + 1)]
            if nearby:
                lidar_stamp = min(nearby, key=lambda x: abs(x-target))
                stamp = self.synced[lidar_stamp].get(name)
                if (stamp is not None and abs(lidar_stamp-target) <= 120_000_000
                        and (self.root / 'raw' / 'image' / name / '{}.jpeg'.format(stamp)).is_file()):
                    return stamp
        stamps = self.images[name]
        index = bisect.bisect_left(stamps, target)
        options = stamps[max(0, index - 1):min(len(stamps), index + 1)]
        if not options:
            return None
        stamp = min(options, key=lambda x: abs(x-target))
        return stamp if abs(stamp-target) <= 120_000_000 else None

    def _boxes(self, frame):
        boxes = []
        for row in self.frames[frame]:
            source_type = int(row['type'])
            if source_type < 0 or source_type == 14:
                continue
            size = [finite(row[k], -1) for k in ('length', 'width', 'height')]
            if min(size) <= 0:
                continue
            boxes.append({
                'id': 'gt:{}:{}'.format(frame, row['id']), 'track_id': row['track_id'],
                'source_type': source_type, 'class': SOURCE_CLASS.get(source_type, 'ignore'),
                'needs_review': source_type in REVIEW_TYPES,
                'confidence': finite(row['type_confidence']),
                'center': [finite(row['center.'+a]) for a in 'xyz'],
                'size': size, 'yaw': finite(row['obj_yaw']),
                'velocity': [finite(row['project_velocity.'+a]) for a in 'xy'],
                'source': 'GT.csv'})
        return boxes

    def data(self, frame):
        if frame not in self.frames:
            abort(404)
        edit_file = self.edits / '{}.json'.format(frame)
        if edit_file.exists():
            content = json.loads(edit_file.read_text())
            boxes, lines = content['boxes'], content['lines']
            reviewed = True
        else:
            boxes = self._boxes(frame)
            lines = read_road_frame(self.road_data, self.road_offsets[self.frame_index[frame]], frame)
            reviewed = False
            content = {}
        deleted_tracks = self.deleted_tracks()
        track_deleted = any(str(box.get('track_id')) in deleted_tracks
                            for box in boxes)
        boxes = [box for box in boxes
                 if str(box.get('track_id')) not in deleted_tracks]
        unique_box_ids(boxes)
        lines = [line for line in lines if line.get('source') != 'road_edge.csv'
                 or line.get('class') != 'boundary_candidate']
        generate_auto_boundaries = (self.regenerate_auto_boundaries
                                    or not content.get('auto_boundaries_reviewed'))
        map_lines = self.map_lines()
        origin_ids = {item['origin_id'] for item in map_lines
                      if item.get('origin_frame') == frame and item.get('origin_id')}
        lines = [line for line in lines
                 if line['id'] not in origin_ids
                 and line.get('source') not in ('global-boundary', 'global-map-line')]
        crossings = self.crossings()
        crossing_origin_ids = {item['origin_id'] for item in crossings
                               if item.get('origin_frame') == frame and item.get('origin_id')}
        lines = [line for line in lines if line.get('source') != 'global-crossing'
                 and line['id'] not in crossing_origin_ids]
        rule_applied = False
        rules = self.rules()
        for box in boxes:
            key = str(box.get('track_id'))
            if key not in rules['tracks']:
                continue
            override = box.get('class_override')
            if override is None and reviewed:
                override = box['class'] not in (
                    SOURCE_CLASS.get(box.get('source_type'), 'ignore'),
                    rules['tracks'][key])
                box['class_override'] = override
            if not override:
                box['class'] = rules['tracks'][key]
                box['needs_review'] = False
                rule_applied = True
        for line in lines:
            if line.get('source') != 'road.bin':
                continue
            key = '{}:{}'.format(line.get('line_id'), line.get('lane_id'))
            if key not in rules['lines']:
                continue
            override = line.get('class_override')
            if override is None and reviewed:
                override = line['class'] not in ('divider', rules['lines'][key])
                line['class_override'] = override
            if not override:
                line['class'] = rules['lines'][key]
                rule_applied = True
        existing = {line['id'] for line in lines}
        for crossing in crossings:
            if crossing['id'] in existing:
                continue
            points = [self.from_global(frame, p) for p in crossing['points']]
            if any(-30 <= p[0] <= 120 and -40 <= p[1] <= 40 for p in points):
                lines.append({'id': crossing['id'], 'class': 'ped_crossing',
                              'source': 'global-crossing', 'points': points})
        for map_line in map_lines:
            points = [self.from_global(frame, p) for p in map_line['points']]
            if (min(p[0] for p in points) <= 120 and max(p[0] for p in points) >= -30
                    and min(p[1] for p in points) <= 40 and max(p[1] for p in points) >= -40):
                lines.append({'id': map_line['id'], 'class': map_line['class'],
                              'source': 'global-map-line', 'points': points})
        # Apply semantic selection after both class rules and global manual
        # map lines, so every explicit boundary takes priority over auto lines.
        if generate_auto_boundaries:
            lines = apply_boundary_policy(lines, frame, self.boundary_offset_m,
                                          self.boundary_policy)
        image_stamps = {name: self._image(frame, name) for name in self.cameras}
        return {'frame': frame, 'timestamp': self.timestamps[frame], 'boxes': boxes,
                'lines': lines, 'images': image_stamps,
                'reviewed': reviewed or rule_applied or track_deleted or any(
                    line.get('source') in ('global-crossing', 'global-map-line')
                    for line in lines),
                'local_reviewed': reviewed}

    def rules(self):
        if self.rules_file.exists():
            return json.loads(self.rules_file.read_text())
        return {'tracks': {}, 'lines': {}}

    def deleted_tracks(self):
        if not self.deleted_tracks_file.exists():
            return set()
        values = json.loads(self.deleted_tracks_file.read_text())
        if not isinstance(values, list) or any(not isinstance(value, str)
                                               for value in values):
            raise ValueError('deleted_tracks.json must contain an array of strings')
        return set(values)

    def write_deleted_tracks(self, values):
        temporary = self.deleted_tracks_file.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(sorted(values), ensure_ascii=False, indent=2))
        os.replace(temporary, self.deleted_tracks_file)

    def track_occurrences(self, track_id):
        count = 0
        frames = 0
        for frame in self.frame_ids:
            edit_file = self.edits / '{}.json'.format(frame)
            if edit_file.exists():
                boxes = json.loads(edit_file.read_text()).get('boxes', [])
            else:
                boxes = self._boxes(frame)
            matches = sum(str(box.get('track_id')) == track_id for box in boxes)
            count += matches
            frames += bool(matches)
        return count, frames

    def delete_track(self, track_id):
        if not isinstance(track_id, str) or not track_id or len(track_id) > 256:
            abort(400, 'invalid track id')
        count, frames = self.track_occurrences(track_id)
        if not count:
            abort(404, 'track id not found')
        deleted = self.deleted_tracks()
        deleted.add(track_id)
        self.write_deleted_tracks(deleted)
        return count, frames

    def restore_track(self, track_id):
        if not isinstance(track_id, str):
            abort(400, 'invalid track id')
        deleted = self.deleted_tracks()
        if track_id not in deleted:
            abort(404, 'track id is not deleted')
        deleted.remove(track_id)
        self.write_deleted_tracks(deleted)

    def crossings(self):
        return json.loads(self.crossings_file.read_text()) if self.crossings_file.exists() else []

    def map_lines(self):
        path = self.map_lines_file if self.map_lines_file.exists() else self.boundaries_file
        if not path.exists():
            return []
        items = json.loads(path.read_text())
        for item in items:
            item.setdefault('class', 'boundary')
        return items

    def write_map_lines(self, items):
        temporary = self.map_lines_file.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(items, ensure_ascii=False, indent=2))
        os.replace(temporary, self.map_lines_file)

    def remove_origin_line(self, frame, identifier):
        local_file = self.edits / '{}.json'.format(frame)
        if not local_file.exists():
            return
        content = json.loads(local_file.read_text())
        filtered = [line for line in content['lines'] if line['id'] != identifier]
        if len(filtered) != len(content['lines']):
            content['lines'] = filtered
            temporary = local_file.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(content, ensure_ascii=False))
            os.replace(temporary, local_file)

    def save_map_line(self, frame, item):
        if frame not in self.poses or item.get('class') not in ('divider', 'boundary'):
            abort(400, 'invalid map-line frame or class')
        points = item.get('points')
        if not isinstance(points, list) or not 2 <= len(points) <= 3000 or any(
                not isinstance(point, list) or len(point) not in (2, 3) or any(
                    not isinstance(value, (int, float)) or not math.isfinite(value)
                    for value in point) for point in points):
            abort(400, 'invalid map-line points')
        map_lines = self.map_lines()
        identifier = item.get('id')
        existing = next((line for line in map_lines if line['id'] == identifier), None)
        if existing is None:
            if isinstance(identifier, str) and identifier.startswith(('boundary:', 'map-line:')):
                abort(404, 'global map line was deleted')
            identifier = 'map-line:{}'.format(uuid.uuid4().hex[:12])
            existing = {'id': identifier, 'origin_frame': frame,
                        'origin_id': str(item.get('id', identifier))}
            map_lines.append(existing)
        existing['class'] = item['class']
        existing['points'] = [self.to_global(frame, point) for point in points]
        self.write_map_lines(map_lines)
        if existing['origin_frame'] == frame and existing['origin_id'] != identifier:
            self.remove_origin_line(frame, existing['origin_id'])
        return identifier

    def delete_map_line(self, identifier):
        map_lines = self.map_lines()
        removed = next((item for item in map_lines if item['id'] == identifier), None)
        if removed is None:
            abort(404)
        self.write_map_lines([item for item in map_lines if item['id'] != identifier])
        return removed

    def restore_map_lines(self, items):
        if not isinstance(items, list) or len(items) > 20:
            abort(400)
        map_lines = self.map_lines()
        existing = {item['id'] for item in map_lines}
        for item in items:
            if not isinstance(item, dict) or not isinstance(item.get('id'), str) \
                    or not item['id'].startswith(('boundary:', 'map-line:')) \
                    or item['id'] in existing \
                    or item.get('class', 'boundary') not in ('divider', 'boundary') \
                    or not isinstance(item.get('origin_frame'), int) \
                    or not isinstance(item.get('origin_id'), str) \
                    or not isinstance(item.get('points'), list) \
                    or not 2 <= len(item['points']) <= 3000 \
                    or any(not isinstance(point, list) or len(point) != 3
                           or any(not isinstance(value, (int, float)) or not math.isfinite(value)
                                  for value in point) for point in item['points']):
                abort(400)
            item.setdefault('class', 'boundary')
            existing.add(item['id'])
            map_lines.append(item)
        self.write_map_lines(map_lines)

    def to_global(self, frame, point):
        translation, quaternion = self.poses[frame]
        rotated = rotate_quaternion((point + [0])[:3], quaternion)
        return [round(rotated[i] + translation[i], 4) for i in range(3)]

    def from_global(self, frame, point):
        translation, quaternion = self.poses[frame]
        result = rotate_quaternion([point[i] - translation[i] for i in range(3)],
                                   quaternion, inverse=True)
        return [round(x, 3) for x in result]

    def global_line_changed(self, frame, local, stored, category):
        if stored is None or stored.get('class', category) != local['class'] \
                or len(stored['points']) != len(local['points']):
            return True
        return any(any(abs(a - b) > 0.001 for a, b in zip(
            (point + [0])[:3], self.from_global(frame, global_point)))
            for point, global_point in zip(local['points'], stored['points']))

    def save_crossing(self, frame, item):
        if frame not in self.poses or item.get('class') != 'ped_crossing':
            abort(400)
        points = item.get('points', [])
        if not isinstance(points, list) or not 4 <= len(points) <= 100 or any(
                not isinstance(p, list) or len(p) not in (2, 3) or any(
                    not isinstance(value, (int, float)) or not math.isfinite(value)
                    for value in p) for p in points):
            abort(400)
        crossings = self.crossings()
        global_points = [self.to_global(frame, p) for p in points]
        identifier = item.get('id')
        existing = next((crossing for crossing in crossings
                         if crossing['id'] == identifier), None)
        if existing is not None:
            existing['points'] = global_points
            self.write_crossings(crossings)
            return identifier
        if isinstance(identifier, str) and identifier.startswith('crossing:'):
            abort(404, 'global crossing was deleted')
        for crossing in crossings:
            if self.same_crossing(crossing['points'], global_points):
                self.remove_origin_line(frame, str(item.get('id')))
                return crossing['id']
        identifier = 'crossing:{}'.format(uuid.uuid4().hex[:12])
        crossings.append({'id': identifier, 'points': global_points,
                          'origin_frame': frame,
                          'origin_id': str(item.get('id', identifier))})
        self.write_crossings(crossings)
        self.remove_origin_line(frame, str(item.get('id', identifier)))
        return identifier

    @staticmethod
    def same_crossing(first, second):
        return len(first) == len(second) and all(
            all(abs(a[i] - b[i]) < 0.05 for i in range(3))
            for a, b in zip(first, second))

    def write_crossings(self, crossings):
        temporary = self.crossings_file.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(crossings, ensure_ascii=False, indent=2))
        os.replace(temporary, self.crossings_file)

    def delete_crossing(self, identifier):
        crossings = self.crossings()
        selected = next((item for item in crossings if item['id'] == identifier), None)
        if selected is None:
            abort(404)
        removed = [item for item in crossings
                   if self.same_crossing(item['points'], selected['points'])]
        deleted = {item['id'] for item in removed}
        self.write_crossings([item for item in crossings if item['id'] not in deleted])
        return removed

    def restore_crossings(self, items):
        if not isinstance(items, list) or len(items) > 20:
            abort(400)
        crossings = self.crossings()
        existing = {item['id'] for item in crossings}
        for item in items:
            if not isinstance(item, dict) or not isinstance(item.get('id'), str) \
                    or not item['id'].startswith('crossing:') or item['id'] in existing \
                    or not isinstance(item.get('points'), list) \
                    or not 4 <= len(item['points']) <= 100 \
                    or any(not isinstance(point, list) or len(point) != 3
                           or any(not isinstance(value, (int, float)) or not math.isfinite(value)
                                  for value in point) for point in item['points']):
                abort(400)
            existing.add(item['id'])
            crossings.append(item)
        self.write_crossings(crossings)

    def save_rule(self, kind, key, category):
        choices = MODEL_CLASSES if kind == 'tracks' else MAP_CLASSES
        if kind not in ('tracks', 'lines') or category not in choices:
            abort(400)
        rules = self.rules()
        rules[kind][key] = category
        temporary = self.rules_file.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(rules, ensure_ascii=False, indent=2))
        os.replace(temporary, self.rules_file)

    def save(self, frame, payload):
        if frame not in self.frames:
            abort(404)
        boxes = payload.get('boxes')
        lines = payload.get('lines')
        if not isinstance(boxes, list) or not isinstance(lines, list):
            abort(400, 'boxes and lines must be arrays')
        deleted_tracks = self.deleted_tracks()
        # Keep globally hidden boxes archived in the per-frame review. This makes
        # restoring a deleted track lossless even if this frame is saved while
        # the track is hidden. A stale client also cannot replace that archive.
        destination = self.edits / '{}.json'.format(frame)
        if destination.exists():
            archived_boxes = json.loads(destination.read_text()).get('boxes', [])
        else:
            archived_boxes = self._boxes(frame)
        boxes = [box for box in boxes
                 if str(box.get('track_id')) not in deleted_tracks]
        boxes.extend(box for box in archived_boxes
                     if str(box.get('track_id')) in deleted_tracks)
        if len(boxes) > 1000 or len(lines) > 500:
            abort(400, 'too many annotations')
        for box in boxes:
            if not isinstance(box, dict) or box.get('class') not in MODEL_CLASSES \
                    or not isinstance(box.get('id'), str) \
                    or not finite_numbers(box.get('center'), 3) \
                    or not isinstance(box.get('yaw'), (int, float)) \
                    or not math.isfinite(box['yaw']):
                abort(400, 'invalid box')
            if not finite_numbers(box.get('size'), 3) or min(box['size']) <= 0:
                abort(400, 'invalid box size')
        for line in lines:
            if not isinstance(line, dict) or line.get('class') not in EDIT_MAP_CLASSES \
                    or not isinstance(line.get('id'), str) \
                    or not isinstance(line.get('points'), list) \
                    or not 2 <= len(line['points']) <= 3000:
                abort(400, 'invalid line')
            if any(not finite_numbers(point, 2) and not finite_numbers(point, 3)
                   for point in line['points']):
                abort(400, 'invalid line point')
        if len({box['id'] for box in boxes}) != len(boxes) \
                or len({line['id'] for line in lines}) != len(lines):
            abort(400, 'duplicate annotation id')
        rules = self.rules()
        for box in boxes:
            if box.get('source') == 'GT.csv' and 'class_override' not in box:
                default = SOURCE_CLASS.get(box.get('source_type'), 'ignore')
                rule = rules['tracks'].get(str(box.get('track_id')))
                box['class_override'] = box['class'] != (rule or default)
        for line in lines:
            if line.get('source') == 'road.bin' and 'class_override' not in line:
                rule = rules['lines'].get('{}:{}'.format(line.get('line_id'),
                                                         line.get('lane_id')))
                line['class_override'] = line['class'] != (rule or 'divider')
        stored_map_lines = {item['id']: item for item in self.map_lines()}
        stored_crossings = {item['id']: item for item in self.crossings()}
        for line in lines:
            if line.get('source') in ('global-boundary', 'global-map-line'):
                if self.global_line_changed(frame, line,
                                            stored_map_lines.get(line['id']),
                                            line['class']):
                    self.save_map_line(frame, line)
            elif line.get('source') == 'global-crossing':
                if self.global_line_changed(frame, line,
                                            stored_crossings.get(line['id']),
                                            'ped_crossing'):
                    self.save_crossing(frame, line)
        temporary = destination.with_suffix('.json.tmp')
        temporary.write_text(json.dumps({'schema': 1, 'frame': frame,
                                         'auto_boundaries_reviewed': True,
                                         'boxes': boxes,
                                         'lines': [line for line in lines
                                                   if line.get('source') not in
                                                   ('global-boundary', 'global-map-line',
                                                    'global-crossing')]},
                                        ensure_ascii=False))
        os.replace(temporary, destination)

    def projected(self, frame, camera_name, annotations=None):
        if camera_name not in self.cameras:
            abort(404)
        data = annotations if annotations is not None else self.data(frame)
        camera = self.cameras[camera_name]
        box_shapes = []
        for box in data['boxes']:
            if box['class'] == 'ignore':
                continue
            corners = [project(p, camera) for p in box_corners(box)]
            if sum(p is not None for p in corners) >= 2:
                box_shapes.append({'id': box['id'], 'track_id': box.get('track_id'),
                                   'class': box['class'], 'points': corners})
        line_shapes = []
        for line in data['lines']:
            if line['class'] == 'ignore':
                continue
            points = [project((p+[0])[:3], camera) for p in line['points'][::max(1,len(line['points'])//150)]]
            line_shapes.append({'id': line['id'], 'class': line['class'], 'points': points})
        return {'boxes': box_shapes, 'lines': line_shapes}

    @lru_cache(maxsize=3)
    def points(self, frame):
        if frame not in self.frames:
            abort(404)
        path = self.result / 'local_motion_pcdbin' / '{}.pcd'.format(frame)
        if not path.is_file():
            return []
        return pcd_bev_points(path)

    def export_labels(self, stride=5):
        """Return reviewed + source labels in the Classlab converter's class vocabulary.

        Geometry remains in vehicle coordinates and is intentionally not called a
        complete training manifest until sensor transforms and camera policy are set.
        """
        output = []
        for frame in self.frame_ids[::stride]:
            data = self.data(frame)
            output.append({'frame': frame, 'timestamp_us': round(data['timestamp']*1e6),
                           'reviewed': data['reviewed'],
                           'boxes': [b for b in data['boxes'] if b['class'] != 'ignore'],
                           'map_annos': {key: [[[p[0],p[1]] for p in line['points']]
                                               for line in data['lines']
                                               if line['class'] == key]
                                         for key in MAP_CLASSES if key != 'ignore'}})
        return output


def make_app(clip):
    app = Flask(__name__)
    static = Path(__file__).parent / 'static'

    @app.get('/')
    def index():
        return send_from_directory(static, 'index.html')

    @app.get('/static/<path:name>')
    def assets(name):
        return send_from_directory(static, name)

    @app.get('/api/meta')
    def meta():
        return jsonify({'clip': clip.root.name, 'frames': clip.frame_ids,
                        'cameras': list(clip.cameras), 'classes': MODEL_CLASSES,
                        'map_classes': EDIT_MAP_CLASSES, 'output': str(clip.edits),
                        'boundary_offset_m': clip.boundary_offset_m,
                        'boundary_policy': clip.boundary_policy,
                        'regenerate_auto_boundaries': clip.regenerate_auto_boundaries})

    @app.get('/api/frame/<int:frame>')
    def frame_data(frame):
        return jsonify(clip.data(frame))

    @app.put('/api/frame/<int:frame>')
    def save_frame(frame):
        clip.save(frame, request.get_json(force=True))
        return jsonify({'saved': True})

    @app.route('/api/projection/<int:frame>/<camera>', methods=['GET', 'POST'])
    def projection(frame, camera):
        if request.method == 'POST':
            annotations = request.get_json(force=True)
            if not isinstance(annotations.get('boxes'), list) or not isinstance(annotations.get('lines'), list):
                abort(400)
            return jsonify(clip.projected(frame, camera, annotations))
        return jsonify(clip.projected(frame, camera))

    @app.get('/api/points/<int:frame>')
    def points(frame):
        return jsonify({'points': clip.points(frame)})

    @app.get('/api/image/<int:frame>/<camera>')
    def image(frame, camera):
        if camera not in clip.cameras or frame not in clip.frames:
            abort(404)
        stamp = clip._image(frame, camera)
        if stamp is None:
            abort(404)
        return send_file(clip.root / 'raw' / 'image' / camera / '{}.jpeg'.format(stamp),
                         mimetype='image/jpeg')

    @app.post('/api/export')
    def export():
        stride = int(request.get_json(silent=True).get('stride', 5)) if request.is_json else 5
        if not 1 <= stride <= 20:
            abort(400)
        output = clip.edits / 'mapped_labels_{}hz.json'.format(round(10/stride, 2))
        output.write_text(json.dumps({'clip': clip.root.name, 'coordinate_frame': 'vehicle',
                                      'stride': stride, 'frames': clip.export_labels(stride)},
                                     ensure_ascii=False))
        return jsonify({'path': str(output)})

    @app.put('/api/rule')
    def save_rule():
        value = request.get_json(force=True)
        clip.save_rule(value.get('kind'), str(value.get('key')), value.get('class'))
        return jsonify({'saved': True})

    @app.delete('/api/track')
    def delete_track():
        value = request.get_json(force=True)
        track_id = value.get('track_id')
        count, frames = clip.delete_track(track_id)
        return jsonify({'track_id': track_id, 'deleted_boxes': count,
                        'affected_frames': frames})

    @app.post('/api/track/restore')
    def restore_track():
        track_id = request.get_json(force=True).get('track_id')
        clip.restore_track(track_id)
        return jsonify({'track_id': track_id, 'restored': True})

    @app.post('/api/crossing/<int:frame>')
    def save_crossing(frame):
        identifier = clip.save_crossing(frame, request.get_json(force=True))
        return jsonify({'id': identifier})

    @app.delete('/api/crossing/<identifier>')
    def delete_crossing(identifier):
        removed = clip.delete_crossing(identifier)
        return jsonify({'deleted': [item['id'] for item in removed], 'items': removed})

    @app.post('/api/crossings/restore')
    def restore_crossings():
        clip.restore_crossings(request.get_json(force=True).get('items'))
        return jsonify({'restored': True})

    @app.post('/api/map-line/<int:frame>')
    def save_map_line(frame):
        identifier = clip.save_map_line(frame, request.get_json(force=True))
        return jsonify({'id': identifier})

    @app.delete('/api/map-line/<identifier>')
    def delete_map_line(identifier):
        removed = clip.delete_map_line(identifier)
        return jsonify({'deleted': [identifier], 'items': [removed]})

    @app.post('/api/map-lines/restore')
    def restore_map_lines():
        clip.restore_map_lines(request.get_json(force=True).get('items'))
        return jsonify({'restored': True})

    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('clip')
    parser.add_argument('--edits-dir', default=None,
                        help='Optional parent folder for per-clip reviews; default: CLIP/reviews')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--boundary-offset-m', type=float, default=0.5,
                        help='Outward boundary fallback offset (default: 0.5 m)')
    parser.add_argument('--boundary-policy', choices=('semantic', 'outer-offset'),
                        default='semantic',
                        help='semantic: solid yellow plus outer-divider fallback; '
                             'outer-offset: legacy extrema')
    parser.add_argument('--regenerate-auto-boundaries', action='store_true',
                        help='preview/regenerate auto lines in saved frames; manual lines are preserved')
    args = parser.parse_args()
    clip = Clip(args.clip, args.edits_dir, args.boundary_offset_m,
                args.boundary_policy, args.regenerate_auto_boundaries)
    make_app(clip).run(host='127.0.0.1', port=args.port, debug=False, threaded=True)
