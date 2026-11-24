"""Generate and review five Stage2 ego-maneuver labels.

Automatic labels are suggestions. Reviewer changes go to reviews/maneuvers.json;
source Classlab files are never modified. Coordinates: x forward, y left.
"""

import bisect
import json
import math
import os
from pathlib import Path

from geometry import rotate_quaternion


MANEUVERS = ('TURN_RIGHT', 'TURN_LEFT', 'KEEP_STRAIGHT',
             'LANE_CHANGE_LEFT', 'LANE_CHANGE_RIGHT')


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _line_id(line):
    if line.get('line_id') is not None:
        return '{}:{}'.format(line['line_id'], line.get('lane_id', ''))
    identifier = str(line.get('id', ''))
    return identifier if identifier.startswith(('map-line:', 'boundary:')) else None


def _lateral_at(line, lookahead=5.0):
    best = None
    for first, second in zip(line.get('points', []), line.get('points', [])[1:]):
        if len(first) < 2 or len(second) < 2:
            continue
        x0, x1 = first[0], second[0]
        if min(x0, x1) <= lookahead <= max(x0, x1) and abs(x1 - x0) > 1e-4:
            ratio = (lookahead - x0) / (x1 - x0)
            return first[1] + ratio * (second[1] - first[1])
        for point in (first, second):
            distance = abs(point[0] - lookahead)
            if best is None or distance < best[0]:
                best = (distance, point[1])
    return best[1] if best and best[0] <= 8.0 else None


def _cross(first, second):
    return first[0] * second[1] - first[1] * second[0]


def _segment_intersection(first, second, third, fourth):
    """Return the trajectory-segment fraction at a proper 2-D intersection."""
    direction = [second[0] - first[0], second[1] - first[1]]
    line_direction = [fourth[0] - third[0], fourth[1] - third[1]]
    denominator = _cross(direction, line_direction)
    if abs(denominator) < 1e-8:
        return None
    offset = [third[0] - first[0], third[1] - first[1]]
    along_trajectory = _cross(offset, line_direction) / denominator
    along_line = _cross(offset, direction) / denominator
    if 1e-4 < along_trajectory <= 1.0 and 0.0 <= along_line <= 1.0:
        return along_trajectory
    return None


class ManeuverLabeler:
    def __init__(self, frames, timestamps, poses, line_provider, reviews_dir,
                 horizon_s=3.0, step_s=0.5):
        self.frames = list(frames)
        self.timestamps = timestamps
        self.poses = poses
        self.line_provider = line_provider
        self.reviews_dir = Path(reviews_dir)
        self.review_file = self.reviews_dir / 'maneuvers.json'
        self.horizon_s = float(horizon_s)
        self.step_s = float(step_s)
        self._times = [timestamps[frame] for frame in self.frames]

    def _nearest_frame(self, timestamp):
        index = bisect.bisect_left(self._times, timestamp)
        indices = range(max(0, index - 1), min(len(self.frames), index + 2))
        best = min(indices, key=lambda value: abs(self._times[value] - timestamp),
                   default=None)
        if best is None or abs(self._times[best] - timestamp) > self.step_s * 0.4:
            return None
        return self.frames[best]

    def _frame_at_or_after(self, timestamp):
        """Return the first frame whose timestamp reaches the requested time."""
        index = bisect.bisect_left(self._times, timestamp)
        if index >= len(self.frames):
            return None
        if self._times[index] - timestamp > self.step_s * 0.4:
            return None
        return self.frames[index]

    def _lane(self, frame):
        candidates = []
        for line in self.line_provider(frame):
            # Stage2 lane-change evidence comes only from detected dividers.
            # A road boundary is a map-training class, not a lane separator.
            if line.get('class') != 'divider':
                continue
            lateral, origin_lateral, identity = (_lateral_at(line),
                                                  _lateral_at(line, 0.0),
                                                  _line_id(line))
            if lateral is not None and origin_lateral is not None and identity is not None:
                candidates.append((lateral, identity, origin_lateral))
        # A line must stay on the same side both at the ego and at lookahead.
        # Otherwise an intersecting/branch line can be mistaken for a lane edge.
        left = min((item for item in candidates
                    if item[0] > 0.2 and item[2] > 0.2),
                   key=lambda item: item[0], default=None)
        right = max((item for item in candidates
                     if item[0] < -0.2 and item[2] < -0.2),
                    key=lambda item: item[0], default=None)
        if left is None or right is None or not 2.0 <= left[0] - right[0] <= 6.0:
            return None
        return {'left': left[1], 'right': right[1],
                'left_y': round(left[0], 3), 'right_y': round(right[0], 3)}

    def _future(self, frame):
        if frame not in self.poses:
            return [], [], []
        origin, quaternion = self.poses[frame]
        trajectory, sampled_frames, headings = [], [], []
        for index in range(1, round(self.horizon_s / self.step_s) + 1):
            future = self._nearest_frame(self.timestamps[frame] + index * self.step_s)
            if future is None or future not in self.poses:
                continue
            translation, future_q = self.poses[future]
            local = rotate_quaternion([translation[i] - origin[i] for i in range(3)],
                                      quaternion, inverse=True)
            forward_global = rotate_quaternion([1, 0, 0], future_q)
            forward_local = rotate_quaternion(forward_global, quaternion, inverse=True)
            trajectory.append([round(local[0], 3), round(local[1], 3)])
            sampled_frames.append(future)
            headings.append(math.atan2(forward_local[1], forward_local[0]))
        return trajectory, sampled_frames, headings

    def _crossed_current_dividers(self, frame, trajectory, current_lane):
        """Find crossings of the current left/right lane dividers.

        Stable road-line IDs are useful context, but cannot prove that the ego
        path stayed inside the lane.  This explicit geometric test is the
        primary evidence for a lane change.
        """
        if current_lane is None or not trajectory:
            return []
        side_by_id = {current_lane['left']: 'LANE_CHANGE_LEFT',
                      current_lane['right']: 'LANE_CHANGE_RIGHT'}
        relevant = {}
        for line in self.line_provider(frame):
            identity = _line_id(line)
            if line.get('class') == 'divider' and identity in side_by_id:
                relevant.setdefault(identity, line)
        path = [[0.0, 0.0]] + trajectory
        crossings = []
        travelled = 0.0
        for segment_index, (first, second) in enumerate(zip(path, path[1:])):
            segment_length = math.hypot(second[0] - first[0], second[1] - first[1])
            for identity, line in relevant.items():
                for third, fourth in zip(line.get('points', []), line.get('points', [])[1:]):
                    fraction = _segment_intersection(first, second, third, fourth)
                    if fraction is None:
                        continue
                    distance = travelled + fraction * segment_length
                    # Ignore numerical contacts at the ego position.
                    if distance >= 0.5:
                        crossings.append({'maneuver': side_by_id[identity],
                                          'line_id': identity,
                                          'distance_m': round(distance, 3),
                                          'time_s': round(
                                              (segment_index + fraction) * self.step_s, 4)})
                    break
            travelled += segment_length
        # A polyline can touch the same trajectory vertex through two adjacent
        # segments. Keep only the first crossing of each physical divider.
        unique = {}
        for crossing in crossings:
            previous = unique.get(crossing['line_id'])
            if previous is None or crossing['distance_m'] < previous['distance_m']:
                unique[crossing['line_id']] = crossing
        return sorted(unique.values(), key=lambda item: item['distance_m'])

    def suggest(self, frame):
        trajectory, future_frames, headings = self._future(frame)
        current = self._lane(frame)
        heading = _wrap(headings[-1]) if headings else 0.0
        crossings = self._crossed_current_dividers(frame, trajectory, current)
        maneuver, valid, reasons = 'KEEP_STRAIGHT', False, []
        if len(trajectory) < round(self.horizon_s / self.step_s):
            reasons.append('future_pose_incomplete')
        elif abs(heading) >= math.radians(20):
            maneuver, valid = ('TURN_LEFT' if heading > 0 else 'TURN_RIGHT'), True
            reasons.append('sustained_heading_change')
        elif len(crossings) == 1 and abs(heading) < math.radians(15):
            maneuver, valid = crossings[0]['maneuver'], True
            reasons.append('future_trajectory_crosses_current_divider')
        elif len(crossings) > 1:
            reasons.append('multiple_current_divider_crossings')
        elif current and abs(heading) <= math.radians(12):
            maneuver, valid = 'KEEP_STRAIGHT', abs(heading) < math.radians(12)
            reasons.append('trajectory_stays_between_detected_dividers')
        else:
            reasons.append('lane_topology_not_reliable')
        suggestion = {
            'frame': frame, 'timestamp': self.timestamps[frame],
            'maneuver': maneuver, 'suggested_maneuver': maneuver,
            'command_valid': bool(valid), 'source': 'auto', 'reason': reasons,
            'future_trajectory_xy': trajectory, 'future_frames': future_frames,
            'heading_change_deg': round(math.degrees(heading), 2),
            'current_lane': current,
            'divider_crossings': crossings,
        }
        reviewed = self.reviews().get(str(frame))
        if reviewed:
            suggestion.update(reviewed)
            suggestion['source'] = 'reviewed'
        command_index = MANEUVERS.index(suggestion['maneuver'])
        suggestion['command_index_5'] = command_index
        suggestion['command_onehot_5'] = [int(i == command_index)
                                           for i in range(len(MANEUVERS))]
        suggestion['planning_mask'] = int(suggestion['command_valid'])
        return suggestion

    def reviews(self):
        if not self.review_file.exists():
            return {}
        content = json.loads(self.review_file.read_text())
        return content if isinstance(content, dict) else {}

    def _write_reviews(self, values):
        self.reviews_dir.mkdir(parents=True, exist_ok=True)
        temporary = self.review_file.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(values, ensure_ascii=False, indent=2))
        os.replace(temporary, self.review_file)

    def save(self, frame, payload):
        maneuver, valid = payload.get('maneuver'), payload.get('command_valid', False)
        if frame not in self.timestamps or maneuver not in MANEUVERS:
            raise ValueError('invalid frame or maneuver')
        if not isinstance(valid, bool):
            raise ValueError('command_valid must be boolean')
        values = self.reviews()
        values[str(frame)] = {'maneuver': maneuver, 'command_valid': valid,
                              'planning_mask': int(valid),
                              'review_note': str(payload.get('review_note', ''))[:500]}
        self._write_reviews(values)
        return self.suggest(frame)

    def apply_lane_change_window(self, review_frame, maneuver, before_s=2.5):
        if review_frame not in self.timestamps:
            raise ValueError('invalid frame')
        if maneuver not in ('LANE_CHANGE_LEFT', 'LANE_CHANGE_RIGHT'):
            raise ValueError('window is only valid for lane changes')
        if not 0 <= before_s <= 10:
            raise ValueError('invalid event window')
        suggestion = self.suggest(review_frame)
        crossing = next((item for item in suggestion.get('divider_crossings', [])
                         if item.get('maneuver') == maneuver), None)
        if crossing is None:
            raise ValueError('future trajectory does not cross the selected divider')
        crossing_time = self.timestamps[review_frame] + crossing['time_s']
        # Completion means the ego has reached the divider centerline.  Using
        # the nearest frame could select the last frame before the crossing.
        crossing_frame = self._frame_at_or_after(crossing_time)
        if crossing_frame is None:
            raise ValueError('divider crossing frame is unavailable')
        # Preserve the original review-window meaning: include history before
        # the frame where the reviewer first observes the lane change, then
        # extend forward only until the geometric crossing is actually reached.
        start_time = self.timestamps[review_frame] - before_s
        values, affected = self.reviews(), []
        for frame in self.frames:
            # Include the first frame at/after the center crossing, but do not
            # extend the event by a fixed amount after it.
            if start_time <= self.timestamps[frame] <= self.timestamps[crossing_frame]:
                values[str(frame)] = {'maneuver': maneuver, 'command_valid': True,
                                      'planning_mask': 1,
                                      'review_note': 'lane_change_until_divider_center_crossing',
                                      'event_anchor_frame': crossing_frame,
                                      'crossing_time_s': round(crossing_time, 6),
                                      'source_review_frame': review_frame}
                affected.append(frame)
        self._write_reviews(values)
        return {'affected_frames': affected,
                'crossing_frame': crossing_frame,
                'crossing_time_s': round(crossing_time, 6),
                'crossed_divider_id': crossing['line_id']}

    def export(self, stride=5):
        output = self.reviews_dir / 'maneuver_labels_{}hz.json'.format(round(10 / stride, 2))
        output.write_text(json.dumps({
            'schema_version': 1, 'coordinate_frame': 'vehicle_x_forward_y_left',
            'horizon_s': self.horizon_s, 'step_s': self.step_s,
            'classes': list(MANEUVERS), 'pkl_modified': False,
            'frames': [self.suggest(frame) for frame in self.frames[::stride]],
        }, ensure_ascii=False, indent=2))
        return output
