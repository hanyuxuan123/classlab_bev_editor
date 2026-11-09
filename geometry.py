"""Small, dependency-light readers for Classlab road polylines and camera projection."""

import math
import struct

import numpy as np


# Solid yellow markings that bound the locally legal driving corridor.
# Yellow dashed (10/12/14) remains divider because it is crossable. Mixed
# solid/dashed yellow (*16) is deliberately excluded until its directed
# solid-side relation to ego can be established reliably.
SEMANTIC_BOUNDARY_TYPES = frozenset((110, 112, 114))


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


def semantic_boundaries(lines, frame_number, distance=0.5):
    """Create semantic barriers plus an outer-road fallback on each side.

    A selected yellow line is used at its measured position, rather than being
    offset. When the outermost usable road line is a different divider, keep it
    as a divider and add a parallel boundary ``distance`` metres farther toward
    the road exterior. Thus an internal no-crossing line and the physical road
    extent can both be represented. The mixed yellow *16 type remains a divider
    because its legal crossing direction cannot be inferred here.
    """
    if not 0 <= distance <= 3:
        raise ValueError('boundary offset must be between 0 and 3 m')
    candidates = _road_candidates(lines)
    semantic = [item for item in candidates
                if item[1].get('lane_type') in SEMANTIC_BOUNDARY_TYPES]
    selected = {}
    for side, valid in (('left', lambda y: y > 0.1),
                        ('right', lambda y: y < -0.1)):
        choices = [item for item in semantic if valid(item[0])]
        if choices:
            selected[side] = min(choices, key=lambda item: abs(item[0]))

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
                'selection': 'nearest-solid-yellow',
                'line_id': source.get('line_id'),
                'lane_id': source.get('lane_id'),
                'lane_type': source.get('lane_type'),
                'source_road_id': source['id'],
                'points': [list(point) for point in points],
            })

        valid = (lambda y: y > 0.1) if side == 'left' else (lambda y: y < -0.1)
        side_candidates = [item for item in candidates if valid(item[0])]
        if not side_candidates:
            continue
        outermost = (max(side_candidates, key=lambda item: item[0])
                     if side == 'left'
                     else min(side_candidates, key=lambda item: item[0]))
        if side in selected and outermost[1]['id'] == selected[side][1]['id']:
            continue
        _, source, points = outermost
        fallback = _offset_boundary(
            source, points, frame_number, side, distance,
            identifier_prefix='road-boundary-semantic-outer',
            source_name='road.bin-semantic-outer',
            selection='outermost-divider-offset')
        if fallback is not None:
            result.append(fallback)
    return result


def apply_boundary_policy(lines, frame_number, distance=0.5, policy='semantic'):
    """Return lines with auto boundaries generated by the requested policy.

    The legacy implementation remains available as ``outer-offset`` for exact
    A/B comparison. Under ``semantic``, a source solid-yellow divider selected
    as a boundary is removed so the same geometry is not trained as two map
    classes. Manual and globally mapped lines are preserved.
    """
    if policy not in ('semantic', 'outer-offset'):
        raise ValueError('unknown boundary policy: {}'.format(policy))
    base = [line for line in lines
            if line.get('source') not in ('road.bin-offset', 'road.bin-semantic',
                                          'road.bin-semantic-outer')]
    if policy == 'outer-offset':
        generated = offset_outer_dividers(base, frame_number, distance)
    else:
        generated = semantic_boundaries(base, frame_number, distance)
        protected_sides = set()
        for line in base:
            if line.get('class') != 'boundary':
                continue
            side = line.get('edge_side')
            if side not in ('left', 'right'):
                nearby = [point[1] for point in line.get('points', [])
                          if len(point) >= 2 and -5 <= point[0] <= 20]
                if nearby:
                    lateral = sum(nearby) / len(nearby)
                    side = 'left' if lateral > 0.1 else 'right' if lateral < -0.1 else None
            if side:
                protected_sides.add(side)
        generated = [line for line in generated
                     if line.get('edge_side') not in protected_sides]
        selected_ids = {line.get('source_road_id') for line in generated
                        if line.get('source') == 'road.bin-semantic'}
        base = [line for line in base if line.get('id') not in selected_ids]
    existing_ids = {line.get('id') for line in base}
    base.extend(line for line in generated if line.get('id') not in existing_ids)
    return base


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
