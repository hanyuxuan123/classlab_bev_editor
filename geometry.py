"""Small, dependency-light readers for Classlab road polylines and camera projection."""

import math
import struct

import numpy as np


# Solid yellow markings that bound the locally legal driving corridor.
# Yellow dashed (10/12/14) remains divider because it is crossable. Mixed
# solid/dashed yellow (*16) is deliberately excluded until its directed
# solid-side relation to ego can be established reliably.
SEMANTIC_BOUNDARY_TYPES = frozenset((110, 112, 114))
BOUNDARY_SCHEMA_VERSION = 9


def _varint(data, pos):
    value = shift = 0
    while True:
        byte = data[pos]
        pos += 1
        value |= (byte & 127) << shift
        if byte < 128:
            return value, pos
        shift += 7


def fields(data, start=0, end=None):
    """Read protobuf wire fields without depending on the vendor .proto."""
    end = len(data) if end is None else end
    pos = start
    while pos < end:
        tag, pos = _varint(data, pos)
        number, wire = tag >> 3, tag & 7
        if wire == 0:
            value, pos = _varint(data, pos)
        elif wire == 1:
            value = data[pos:pos + 8]
            pos += 8
        elif wire == 2:
            size, pos = _varint(data, pos)
            value = (pos, pos + size)
            pos += size
        elif wire == 5:
            value = data[pos:pos + 4]
            pos += 4
        else:
            raise ValueError("unsupported protobuf wire type {}".format(wire))
        yield number, wire, value


def frame_offsets(data):
    return [value for number, wire, value in fields(data) if number == 3 and wire == 2]


def _signed64(value):
    return value if value < 1 << 63 else value - (1 << 64)


def read_road_frame(data, offset, frame_number):
    """Decode the observed road.bin layout: frame.3 -> line.2 -> point.5."""
    lines = []
    for number, wire, region in fields(data, *offset):
        if number != 2 or wire != 2:
            continue
        line_id = lane_id = lane_type = None
        points = []
        for field, kind, value in fields(data, *region):
            if kind == 0 and field == 1:
                line_id = _signed64(value)
            elif kind == 0 and field == 2:
                lane_id = _signed64(value)
            elif kind == 0 and field == 3:
                lane_type = _signed64(value)
            elif kind == 2 and field == 5:
                xyz = {}
                for axis, encoding, scalar in fields(data, *value):
                    if encoding == 5 and axis in (1, 2, 3):
                        xyz[axis] = struct.unpack('<f', scalar)[0]
                if 1 in xyz and 2 in xyz:
                    points.append([round(xyz[1], 3), round(xyz[2], 3), round(xyz.get(3, 0.0), 3)])
        if len(points) >= 2:
            lines.append({"id": "road:{}:{}:{}".format(frame_number, line_id, lane_id),
                          "class": "divider", "source": "road.bin", "line_id": line_id,
                          "lane_id": lane_id, "lane_type": lane_type, "points": points})
    return lines


def offset_outer_dividers(lines, frame_number, distance=0.5):
    """Offset the outer road.bin lines outward, without extending their ends."""
    if not 0 <= distance <= 3:
        raise ValueError('boundary offset must be between 0 and 3 m')
    candidates = []
    for line in lines:
        if line.get('source') != 'road.bin' or len(line.get('points', [])) < 2:
            continue
        points = sorted(line['points'], key=lambda point: point[0])
        if points[-1][0] - points[0][0] < 8:
            continue
        probes = []
        for x in (0, 10, 20):
            for a, b in zip(points, points[1:]):
                if a[0] <= x <= b[0] and b[0] > a[0]:
                    t = (x - a[0]) / (b[0] - a[0])
                    probes.append(a[1] + t * (b[1] - a[1]))
                    break
        if len(probes) >= 2:
            candidates.append((sum(probes) / len(probes), line, points))
    if len(candidates) < 2:
        return []
    left = max(candidates, key=lambda item: item[0])
    right = min(candidates, key=lambda item: item[0])
    if left[1]['id'] == right[1]['id'] or left[0] - right[0] < 1:
        return []
    result = []
    for side, (_, source, points) in (('left', left), ('right', right)):
        if source.get('class') == 'boundary':
            continue
        shifted = []
        for index, point in enumerate(points):
            before = points[max(0, index - 1)]
            after = points[min(len(points) - 1, index + 1)]
            dx, dy = after[0] - before[0], after[1] - before[1]
            length = math.hypot(dx, dy)
            if length < 1e-6:
                continue
            sign = 1 if side == 'left' else -1
            shifted.append([round(point[0] - sign * distance * dy / length, 3),
                            round(point[1] + sign * distance * dx / length, 3),
                            round(point[2] if len(point) > 2 else 0.0, 3)])
        if len(shifted) >= 2:
            result.append({'id': 'road-boundary:{}:{}'.format(frame_number, side),
                           'class': 'boundary', 'source': 'road.bin-offset',
                           'edge_side': side, 'offset_m': distance,
                           'source_road_id': source['id'], 'points': shifted})
    return result


def _road_candidates(lines):
    """Return long road.bin lines with a stable lateral position ahead."""
    candidates = []
    for line in lines:
        if line.get('source') != 'road.bin' or line.get('class') != 'divider' \
                or line.get('class_override', False) \
                or len(line.get('points', [])) < 2:
            continue
        points = sorted(line['points'], key=lambda point: point[0])
        if points[-1][0] - points[0][0] < 8:
            continue
        probes = []
        for x in (0, 10, 20):
            for a, b in zip(points, points[1:]):
                if a[0] <= x <= b[0] and b[0] > a[0]:
                    t = (x - a[0]) / (b[0] - a[0])
                    probes.append(a[1] + t * (b[1] - a[1]))
                    break
        if len(probes) >= 2:
            candidates.append((sum(probes) / len(probes), line, points))
    return candidates


def _offset_boundary(source, points, frame_number, side, distance,
                     identifier_prefix='road-boundary',
                     source_name='road.bin-offset', selection=None):
    """Build a boundary parallel to a source line on its road-exterior side."""
    shifted = []
    for index, point in enumerate(points):
        before = points[max(0, index - 1)]
        after = points[min(len(points) - 1, index + 1)]
        dx, dy = after[0] - before[0], after[1] - before[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        sign = 1 if side == 'left' else -1
        shifted.append([round(point[0] - sign * distance * dy / length, 3),
                        round(point[1] + sign * distance * dx / length, 3),
                        round(point[2] if len(point) > 2 else 0.0, 3)])
    if len(shifted) < 2:
        return None
    result = {
        'id': '{}:{}:{}'.format(identifier_prefix, frame_number, side),
        'class': 'boundary',
        'source': source_name,
        'edge_side': side,
        'offset_m': distance,
        'line_id': source.get('line_id'),
        'lane_id': source.get('lane_id'),
        'lane_type': source.get('lane_type'),
        'source_road_id': source['id'],
        'points': shifted,
    }
    if selection is not None:
        result['selection'] = selection
    return result


def road_edge_lines(row, frame_number):
    """Convert one road_edge.csv row into left/right polynomial candidates."""
    result = []
    for side, prefix in (('left', 'left_edge'), ('right', 'righ_edge')):
        if str(row.get(prefix + '.isValid', '')).lower() != 'true':
            continue
        try:
            start = max(-10.0, float(row[prefix + '.rangeN']))
            stop = min(120.0, float(row[prefix + '.rangeP']))
            coefficients = [float(row[prefix + name]) for name in (
                '.zeroOrderCoeff', '.firstOrderCoeff', '.secondOrderCoeff')]
            near = float(row[prefix + '.thirdOrderCoeffNear'])
            far = float(row[prefix + '.thirdOrderCoeffFar'])
            dividing = float(row[prefix + '.dividingPoint'])
            use_far = str(row.get(prefix + '.fitFarFlag', '')).lower() == 'true'
        except (KeyError, TypeError, ValueError):
            continue
        if stop - start < 8:
            continue
        switch_jump = (abs((far - near) * dividing**3)
                       if use_far and start < dividing < stop else 0.0)
        points = []
        x = math.ceil(start)
        while x <= math.floor(stop):
            third = far if use_far and x > dividing else near
            y = coefficients[0] + coefficients[1] * x + coefficients[2] * x*x + third * x*x*x
            if math.isfinite(y) and abs(y) <= 100:
                points.append([float(x), round(y, 3), 0.0])
            x += 1
        if len(points) >= 9:
            result.append({
                'id': 'road-edge-candidate:{}:{}'.format(frame_number, side),
                'class': 'boundary_candidate', 'source': 'road_edge.csv',
                'edge_side': side, 'points': points,
                'fit_switch_jump_m': round(switch_jump, 3),
            })
    return result


def semantic_boundaries_v2(lines, frame_number, distance=0.5, ambiguity_m=1.0,
                           return_status=False):
    """Create auto boundaries only where solid-yellow evidence is unambiguous."""
    if not 0 <= distance <= 3:
        raise ValueError('boundary offset must be between 0 and 3 m')
    if not 0 < ambiguity_m <= 10:
        raise ValueError('boundary ambiguity threshold must be in (0, 10] m')
    candidates = _road_candidates(lines)
    semantic = [item for item in candidates
                if item[1].get('lane_type') in SEMANTIC_BOUNDARY_TYPES]
    selected = {}
    status = {'policy': 'semantic-v2', 'ambiguity_m': ambiguity_m,
              'ambiguous_sides': [], 'absent_sides': [], 'sides': {}}
    for side, valid in (('left', lambda y: y > 0.1),
                        ('right', lambda y: y < -0.1)):
        choices = [item for item in semantic if valid(item[0])]
        lateral = sorted(abs(item[0]) for item in choices)
        side_status = {'candidate_count': len(choices)}
        if not choices:
            side_status['state'] = 'absent'
            status['absent_sides'].append(side)
        elif len(choices) == 1:
            side_status['state'] = 'single'
            side_status['spread_m'] = 0.0
            selected[side] = choices[0]
        else:
            spread = lateral[-1] - lateral[0]
            side_status['spread_m'] = round(spread, 3)
            if spread >= ambiguity_m:
                side_status['state'] = 'ambiguous'
                status['ambiguous_sides'].append(side)
            else:
                side_status['state'] = 'clustered'
                selected[side] = max(choices, key=lambda item: abs(item[0]))
        status['sides'][side] = side_status

    result = []
    for side in ('left', 'right'):
        if side in selected:
            _, source, points = selected[side]
            result.append({
                'id': 'road-boundary-semantic:{}:{}'.format(frame_number, side),
                'class': 'boundary',
                'source': 'road.bin-semantic',
                'edge_side': side,
                'offset_m': 0.0,
                'selection': ('single-solid-yellow' if
                              status['sides'][side]['state'] == 'single' else
                              'clustered-outermost-solid-yellow'),
                'line_id': source.get('line_id'),
                'lane_id': source.get('lane_id'),
                'lane_type': source.get('lane_type'),
                'source_road_id': source['id'],
                'points': [list(point) for point in points],
            })
    status['valid'] = not status['ambiguous_sides']
    return (result, status) if return_status else result


def _lateral_at(points, x):
    points = sorted(points, key=lambda point: point[0])
    for first, second in zip(points, points[1:]):
        if first[0] <= x <= second[0] and second[0] > first[0]:
            ratio = (x - first[0]) / (second[0] - first[0])
            return first[1] + ratio * (second[1] - first[1])
    return None


def _stable_on_side(points, side):
    """Reject a line that crosses the ego axis anywhere in its forward span.

    ``road.bin`` occasionally contains long, internally smooth polylines that
    drift from one side of the road to the other.  A near-field-only probe
    incorrectly treated those lines as stable hard-yellow/outer-edge evidence.
    """
    forward = [point[1] for point in points
               if len(point) >= 2 and point[0] >= 0 and math.isfinite(point[1])]
    if len(forward) < 2:
        return False
    if side == 'left':
        return min(forward) > 0.1
    return max(forward) < -0.1


def _stable_near_side(points, side, max_x=20.0):
    """Check semantic lane side locally without rejecting a curved road.

    In vehicle coordinates a correct boundary on a right-hand bend can cross
    y=0 farther ahead.  Lane-type evidence is explicit, so only the near-field
    relation to the ego corridor should decide left versus right.
    """
    lateral = [point[1] for point in points
               if len(point) >= 2 and 0 <= point[0] <= max_x
               and math.isfinite(point[1])]
    if len(lateral) < 2:
        return False
    if side == 'left':
        return min(lateral) > 0.1
    return max(lateral) < -0.1


def _validate_road_edge(line, side, road_candidates):
    points = line.get('points', [])
    if len(points) < 2 or points[-1][0] - points[0][0] < 15:
        return False, 'short'
    if line.get('fit_switch_jump_m', 0.0) > 0.75:
        return False, 'discontinuous-fit-switch'
    if not _stable_on_side(points, side):
        return False, 'crosses-ego-axis'
    at_zero = _lateral_at(points, 0)
    if at_zero is None or (side == 'left' and at_zero <= 1.0) \
            or (side == 'right' and at_zero >= -1.0):
        return False, 'wrong-side-near-ego'
    comparisons = []
    edge_values = []
    max_x = min(80, int(math.floor(points[-1][0] / 5.0)) * 5)
    for x in range(0, max_x + 1, 5):
        edge = _lateral_at(points, x)
        roads = [_lateral_at(candidate[2], x) for candidate in road_candidates]
        roads = [value for value in roads if value is not None]
        if edge is None or not roads:
            continue
        outer = max(roads) if side == 'left' else min(roads)
        gap = edge - outer if side == 'left' else outer - edge
        comparisons.append(0.2 <= gap <= 8.0)
        edge_values.append(edge)
    if len(comparisons) < 3:
        return False, 'insufficient-overlap'
    if sum(comparisons) / len(comparisons) < 0.8:
        return False, 'not-outside-road-lines'
    if any(abs(second - first) > 4.0 for first, second in zip(
            edge_values, edge_values[1:])):
        return False, 'unstable-curve'
    return True, 'validated'


def semantic_boundaries(lines, frame_number, distance=0.5, ambiguity_m=1.0,
                        return_status=False):
    """Semantic V3: preserve dividers and close both sides of the ego corridor."""
    if not 0 <= distance <= 3:
        raise ValueError('boundary offset must be between 0 and 3 m')
    if not 0 < ambiguity_m <= 10:
        raise ValueError('solid-yellow cluster threshold must be in (0, 10] m')
    candidates = _road_candidates(lines)
    solid = [item for item in candidates
             if item[1].get('lane_type') in SEMANTIC_BOUNDARY_TYPES]
    edge_candidates = [line for line in lines
                       if line.get('source') == 'road_edge.csv']
    result = []
    status = {'policy': 'semantic', 'schema': BOUNDARY_SCHEMA_VERSION,
              'solid_yellow_cluster_m': ambiguity_m,
              'sides': {}, 'unresolved_sides': [], 'ambiguous_sides': []}
    for side, valid in (('left', lambda y: y > 0.1),
                        ('right', lambda y: y < -0.1)):
        choices = sorted((item for item in solid if valid(item[0])),
                         key=lambda item: abs(item[0]))
        stable_choices = [item for item in choices
                          if _stable_near_side(item[2], side)]
        side_status = {
            'solid_yellow_candidates': len(choices),
            'stable_solid_yellow_candidates': len(stable_choices),
        }
        generated = None
        if stable_choices:
            first_distance = abs(stable_choices[0][0])
            cluster = [item for item in stable_choices
                       if abs(abs(item[0]) - first_distance) < ambiguity_m]
            _, source, points = min(cluster, key=lambda item: abs(item[0]))
            generated = {
                'id': 'road-boundary-semantic:{}:{}'.format(frame_number, side),
                'class': 'boundary', 'source': 'road.bin-semantic',
                'edge_side': side, 'offset_m': 0.0,
                'selection': 'first-solid-yellow-cluster-near-edge',
                'cluster_size': len(cluster),
                'line_id': source.get('line_id'), 'lane_id': source.get('lane_id'),
                'lane_type': source.get('lane_type'),
                'source_road_id': source['id'],
                'points': [list(point) for point in points],
            }
            side_status.update(state='hard-divider', source='solid-yellow-cluster',
                               cluster_size=len(cluster), valid=True)
        else:
            edge = next((line for line in edge_candidates
                         if line.get('edge_side') == side), None)
            accepted, reason = ((False, 'missing') if edge is None else
                                _validate_road_edge(edge, side, candidates))
            side_status['road_edge_check'] = reason
            if accepted:
                generated = dict(edge)
                generated.update(
                    id='road-boundary-edge:{}:{}'.format(frame_number, side),
                    **{'class': 'boundary', 'source': 'road_edge.csv-validated',
                       'selection': 'validated-road-edge'})
                side_status.update(state='road-edge', source='validated-road-edge',
                                   valid=True)
            else:
                # Do not manufacture a drivable-area boundary by offsetting an
                # arbitrary outer road.bin divider.  At junctions that line can
                # belong to another branch.  Lane-type semantic boundaries and
                # independently validated road-edge boundaries remain valid.
                side_status.update(state='unresolved', source=None, valid=False)
                status['unresolved_sides'].append(side)
        if generated is not None:
            result.append(generated)
        status['sides'][side] = side_status
    # ``valid`` means every emitted boundary is backed by an accepted source
    # and may enter training. ``complete`` separately records whether both
    # corridor sides were recovered; a missing side must not discard a valid
    # lane-type/road-edge boundary on the other side.
    status['complete'] = not status['unresolved_sides'] \
        and not status['ambiguous_sides']
    status['valid'] = bool(result) and not status['ambiguous_sides']
    return (result, status) if return_status else result


def _boundary_side(line):
    side = line.get('edge_side')
    if side in ('left', 'right'):
        return side
    lateral = [point[1] for point in line.get('points', [])
               if len(point) >= 2 and -5 <= point[0] <= 20]
    if not lateral:
        lateral = [point[1] for point in line.get('points', []) if len(point) >= 2]
    mean = sum(lateral) / len(lateral) if lateral else 0.0
    return 'left' if mean > 0.1 else 'right' if mean < -0.1 else None


def _point_segment_distance(point, first, second):
    dx, dy = second[0] - first[0], second[1] - first[1]
    length2 = dx * dx + dy * dy
    if length2 <= 1e-12:
        return math.hypot(point[0] - first[0], point[1] - first[1])
    t = max(0.0, min(1.0, ((point[0] - first[0]) * dx +
                           (point[1] - first[1]) * dy) / length2))
    return math.hypot(point[0] - first[0] - t * dx,
                      point[1] - first[1] - t * dy)


def _subtract_manual_coverage(line, manual_lines, buffer_m):
    """Split an auto line into runs outside the manual-line spatial buffer."""
    if not manual_lines:
        return [line]
    segments = [(a, b) for manual in manual_lines
                for a, b in zip(manual.get('points', []),
                                manual.get('points', [])[1:])]
    keep = [all(_point_segment_distance(point, a, b) > buffer_m
                for a, b in segments) for point in line['points']]
    runs, current = [], []
    for point, valid in zip(line['points'], keep):
        if valid:
            current.append(point)
        elif len(current) >= 2:
            runs.append(current)
            current = []
        else:
            current = []
    if len(current) >= 2:
        runs.append(current)
    result = []
    for index, points in enumerate(runs):
        item = dict(line)
        item['id'] = '{}:segment{}'.format(line['id'], index)
        item['points'] = points
        result.append(item)
    return result


def apply_boundary_policy(lines, frame_number, distance=0.5, policy='semantic',
                          ambiguity_m=1.0, manual_buffer_m=1.0,
                          return_status=False):
    """Return lines with auto boundaries generated by the requested policy.

    ``semantic`` keeps a solid-yellow line as ``divider`` when
    duplicating its geometry as ``boundary``. ``semantic-v2`` preserves the old
    mutually-exclusive behavior for rollback/A-B comparison.
    """
    if policy not in ('semantic', 'semantic-v2', 'outer-offset'):
        raise ValueError('unknown boundary policy: {}'.format(policy))
    if not 0 <= manual_buffer_m <= 5:
        raise ValueError('manual boundary buffer must be between 0 and 5 m')
    auto_sources = ('road.bin-offset', 'road.bin-semantic',
                    'road.bin-semantic-outer', 'road_edge.csv-validated')
    base = [line for line in lines if line.get('source') not in auto_sources]
    if policy == 'outer-offset':
        generated = offset_outer_dividers(base, frame_number, distance)
        status = {'policy': policy, 'valid': True, 'ambiguous_sides': [],
                  'absent_sides': [], 'sides': {}}
    else:
        boundary_fn = semantic_boundaries_v2 if policy == 'semantic-v2' else semantic_boundaries
        generated, status = boundary_fn(
            base, frame_number, distance, ambiguity_m, return_status=True)
        manual_by_side = {'left': [], 'right': []}
        for line in base:
            if line.get('class') == 'boundary':
                side = _boundary_side(line)
                if side:
                    line['edge_side'] = side
                    manual_by_side[side].append(line)
        status['manual_sides'] = [side for side, values in manual_by_side.items()
                                  if values]
        for side in status.get('sides', {}):
            if manual_by_side[side]:
                status['sides'][side].update(
                    state='manual', source='manual/global', valid=True)
                if side in status.get('unresolved_sides', []):
                    status['unresolved_sides'].remove(side)
                if side in status.get('ambiguous_sides', []):
                    status['ambiguous_sides'].remove(side)
        if policy == 'semantic':
            status['complete'] = not status.get('unresolved_sides', []) \
                and not status.get('ambiguous_sides', [])
            status['valid'] = bool(generated) or any(manual_by_side.values())
            status['valid'] = status['valid'] \
                and not status.get('ambiguous_sides', [])
        else:
            status['valid'] = not status.get('unresolved_sides', []) \
                and not status.get('ambiguous_sides', [])
        if policy == 'semantic-v2':
            selected_ids = {line.get('source_road_id') for line in generated
                            if line.get('source') == 'road.bin-semantic'}
            base = [line for line in base if line.get('id') not in selected_ids]
        generated = [segment for line in generated
                     for segment in _subtract_manual_coverage(
                         line, manual_by_side[line['edge_side']], manual_buffer_m)]
    existing_ids = {line.get('id') for line in base}
    base.extend(line for line in generated if line.get('id') not in existing_ids)
    base = [line for line in base if line.get('source') != 'road_edge.csv']
    return (base, status) if return_status else base


def rotation_xyz(angles):
    roll, pitch, yaw = angles
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
            [-sp, cp*sr, cp*cr]]


def project(point, camera):
    ext, k, d = camera["ext"], camera["K"], camera["D"]
    rot = rotation_xyz([ext["roll"], ext["pitch"], ext["yaw"]])
    delta = [point[i] - ext[axis] for i, axis in enumerate("xyz")]
    x, y, z = [sum(rot[j][i]*delta[j] for j in range(3)) for i in range(3)]
    if z <= 0.05:
        return None
    x, y = x/z, y/z
    r2 = x*x+y*y
    if not math.isfinite(r2):
        return None
    k1,k2,p1,p2,k3,k4,k5,k6 = (list(d)+[0]*8)[:8]
    denominator = 1+k4*r2+k5*r2*r2+k6*r2*r2*r2
    if not math.isfinite(denominator) or abs(denominator) < 1e-12:
        return None
    radial = (1+k1*r2+k2*r2*r2+k3*r2*r2*r2)/denominator
    xd = x*radial+2*p1*x*y+p2*(r2+2*x*x)
    yd = y*radial+p1*(r2+2*y*y)+2*p2*x*y
    u, v = k[0]*xd+k[2], k[4]*yd+k[5]
    return [round(u, 2), round(v, 2)] if math.isfinite(u) and math.isfinite(v) else None


def box_corners(box):
    x,y,z = box["center"]
    length,width,height = box["size"]
    yaw = box["yaw"]
    c,s = math.cos(yaw),math.sin(yaw)
    out=[]
    for dz in (-height/2,height/2):
        for dx,dy in ((-length/2,-width/2),(length/2,-width/2),
                      (length/2,width/2),(-length/2,width/2)):
            out.append([x+c*dx-s*dy,y+s*dx+c*dy,z+dz])
    return out


def rotate_quaternion(point, quaternion, inverse=False):
    """Rotate xyz using an [x,y,z,w] quaternion."""
    x,y,z,w = quaternion
    if inverse:
        x,y,z = -x,-y,-z
    ux,uy,uz = x,y,z
    px,py,pz = point
    tx,ty,tz = (2*(uy*pz-uz*py),2*(uz*px-ux*pz),2*(ux*py-uy*px))
    return [px+w*tx+uy*tz-uz*ty,
            py+w*ty+uz*tx-ux*tz,
            pz+w*tz+ux*ty-uy*tx]


def lzf_decompress(source, expected_size):
    """Decode PCL's LZF block; back-references may overlap the output."""
    out = bytearray()
    index = 0
    while index < len(source):
        control = source[index]
        index += 1
        if control < 32:
            length = control + 1
            out.extend(source[index:index + length])
            index += length
        else:
            length = control >> 5
            reference = len(out) - ((control & 31) << 8) - 1
            if length == 7:
                length += source[index]
                index += 1
            reference -= source[index]
            index += 1
            length += 2
            if reference < 0:
                raise ValueError('invalid LZF back-reference')
            for _ in range(length):
                out.append(out[reference])
                reference += 1
    if len(out) != expected_size:
        raise ValueError('LZF output size mismatch')
    return out


def pcd_bev_points(path, stride=24):
    """Return a sparse XY background from a Classlab binary_compressed PCD."""
    with open(path, 'rb') as handle:
        header = []
        while True:
            line = handle.readline().decode('ascii').strip()
            header.append(line)
            if line.startswith('DATA '):
                break
        if header[-1] != 'DATA binary_compressed':
            raise ValueError('expected binary_compressed PCD')
        fields_line = next(line for line in header if line.startswith('FIELDS '))
        size_line = next(line for line in header if line.startswith('SIZE '))
        count = int(next(line for line in header if line.startswith('POINTS ')).split()[1])
        field_names = fields_line.split()[1:]
        field_sizes = list(map(int, size_line.split()[1:]))
        compressed_size, uncompressed_size = struct.unpack('<II', handle.read(8))
        raw = lzf_decompress(handle.read(compressed_size), uncompressed_size)
    offsets = {}
    start = 0
    for name, size in zip(field_names, field_sizes):
        offsets[name] = start
        start += size*count
    x = np.frombuffer(raw, dtype='<f4', count=count, offset=offsets['x'])[::stride]
    y = np.frombuffer(raw, dtype='<f4', count=count, offset=offsets['y'])[::stride]
    mask = np.isfinite(x) & np.isfinite(y) & (x > -35) & (x < 130) & (np.abs(y) < 55)
    return np.stack([x[mask], y[mask]], axis=1).round(2).tolist()
