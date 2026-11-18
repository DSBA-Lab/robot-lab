"""Convert robot descriptions (URDF + STL/DAE) into compact articulated GLB files.

Node layout per joint:  <origin node, static>  ->  <joint node, named after the joint, rotated at runtime>
Joint metadata (axis, limits, mimic) is stored in the joint node's `extras`.
"""
import json, math, os, re, struct, sys
import xml.etree.ElementTree as ET
import numpy as np
import fast_simplification

SRC, OUT = sys.argv[1], sys.argv[2]
BS = float(os.environ.get('BUDGET_SCALE', '1'))  # <1 builds low-detail copies for testing
CREASE = math.radians(38)


# ---------- math ----------
def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def mat_quat(m):
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1) * 2; q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2]) * 2; q = [s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2]) * 2; q = [(m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]) * 2; q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s]
    n = math.sqrt(sum(c * c for c in q))
    return [round(c / n, 7) for c in q]


def fl(s, n=3, d=0.0):
    v = [float(x) for x in s.split()] if s else [d] * n
    return v


# ---------- mesh loading ----------
def load_stl(path):
    b = open(path, 'rb').read()
    n = struct.unpack_from('<I', b, 80)[0]
    if 84 + 50 * n != len(b):
        raise ValueError('ascii STL not supported: ' + path)
    rec = np.frombuffer(b, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=n, offset=84)
    return rec['v'].reshape(-1, 3).astype(np.float64)  # triangle soup


def load_dae(path):
    """Returns list of (rgba, soup Nx3) with node matrices applied; the file's up_axis is ignored (as RViz does)."""
    root = ET.parse(path).getroot()
    ns = re.match(r'\{.*\}', root.tag).group(0)
    q = lambda s: s.replace('c:', ns)
    unit = root.find(q('c:asset/c:unit'))
    meter = float(unit.get('meter', 1)) if unit is not None else 1.0
    effects = {}
    for e in root.iter(q('c:effect')):
        c = None
        for d in e.iter(q('c:diffuse')):
            c = d.find(q('c:color'))
        effects[e.get('id')] = [float(x) for x in c.text.split()] if c is not None else [0.7, 0.7, 0.7, 1]
    mats = {}
    for m in root.iter(q('c:material')):
        ie = m.find(q('c:instance_effect'))
        mats[m.get('id')] = effects.get(ie.get('url')[1:], [0.7, 0.7, 0.7, 1])
    geoms = {}
    for g in root.iter(q('c:geometry')):
        mesh = g.find(q('c:mesh'))
        srcs = {}
        for s in mesh.findall(q('c:source')):
            fa = s.find(q('c:float_array'))
            acc = s.find(q('c:technique_common/c:accessor'))
            stride = int(acc.get('stride', 3))
            srcs[s.get('id')] = np.array(fa.text.split(), dtype=np.float64).reshape(-1, stride)
        verts = {}
        for v in mesh.findall(q('c:vertices')):
            for i in v.findall(q('c:input')):
                if i.get('semantic') == 'POSITION':
                    verts[v.get('id')] = srcs[i.get('source')[1:]]
        prims = []
        for p in mesh:
            tag = p.tag.replace(ns, '')
            if tag not in ('triangles', 'polylist'):
                continue
            inputs = p.findall(q('c:input'))
            stride = max(int(i.get('offset')) for i in inputs) + 1
            voff = [int(i.get('offset')) for i in inputs if i.get('semantic') == 'VERTEX'][0]
            vsrc = [i.get('source')[1:] for i in inputs if i.get('semantic') == 'VERTEX'][0]
            idx = np.array(p.find(q('c:p')).text.split(), dtype=np.int64).reshape(-1, stride)[:, voff]
            if tag == 'polylist':
                vc = np.array(p.find(q('c:vcount')).text.split(), dtype=np.int64)
                if not np.all(vc == 3):
                    tris, k = [], 0
                    for c in vc:
                        for j in range(1, c - 1):
                            tris += [idx[k], idx[k + j], idx[k + j + 1]]
                        k += c
                    idx = np.array(tris, dtype=np.int64)
            prims.append((p.get('material'), verts[vsrc][idx][:, :3]))
        geoms[g.get('id')] = prims
    out = []

    def walk(node, M):
        for c in node:
            tag = c.tag.replace(ns, '')
            if tag == 'matrix':
                M = M @ np.array(c.text.split(), dtype=np.float64).reshape(4, 4)
            elif tag == 'translate':
                T = np.eye(4); T[:3, 3] = fl(c.text); M = M @ T
            elif tag == 'scale':
                M = M @ np.diag(fl(c.text) + [1])
            elif tag == 'rotate':
                x, y, z, a = fl(c.text, 4); a = math.radians(a); k = np.array([x, y, z]) / np.linalg.norm([x, y, z])
                K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
                R = np.eye(4); R[:3, :3] = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K; M = M @ R
        for ig in node.findall(q('c:instance_geometry')):
            bind = {im.get('symbol'): im.get('target')[1:] for im in ig.iter(q('c:instance_material'))}
            for sym, soup in geoms[ig.get('url')[1:]]:
                pts = soup @ M[:3, :3].T + M[:3, 3]
                if np.linalg.det(M[:3, :3]) < 0:
                    pts = pts.reshape(-1, 3, 3)[:, ::-1].reshape(-1, 3)
                out.append((mats.get(bind.get(sym, sym), [0.7, 0.7, 0.7, 1]), pts * meter))
        for c in node.findall(q('c:node')):
            walk(c, M)

    for vs in root.iter(q('c:visual_scene')):
        for n in vs.findall(q('c:node')):
            walk(n, np.eye(4))
    return out


# ---------- mesh processing ----------
def weld(soup, tol=1e-5):
    key = np.round(soup / tol).astype(np.int64)
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    inv = inv.reshape(-1)
    v = soup[first]
    f = inv.reshape(-1, 3)
    ok = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    return v, f[ok]


def simplify(v, f, target):
    if len(f) <= target:
        return v, f
    v2, f2 = fast_simplification.simplify(v.astype(np.float32), f.astype(np.int32), target_count=int(target), agg=5)
    v2 = np.asarray(v2, dtype=np.float64); f2 = np.asarray(f2, dtype=np.int64)
    ok = (f2[:, 0] != f2[:, 1]) & (f2[:, 1] != f2[:, 2]) & (f2[:, 0] != f2[:, 2])
    return v2, f2[ok]


def creased(v, f):
    """Split vertices across sharp edges; returns positions, normals (float), indices."""
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    fn = np.cross(b - a, c - a)                      # area-weighted face normals
    ar = np.linalg.norm(fn, axis=1)
    keep = ar > 1e-14
    f, fn, ar = f[keep], fn[keep], ar[keep]
    un = fn / ar[:, None]
    nf = len(f)
    corner_v = f.reshape(-1)
    corner_f = np.repeat(np.arange(nf), 3)
    order = np.argsort(corner_v, kind='stable')
    cv, cf = corner_v[order], corner_f[order]
    starts = np.flatnonzero(np.r_[True, cv[1:] != cv[:-1]])
    ends = np.r_[starts[1:], len(cv)]
    cosc = math.cos(CREASE)
    cn = np.zeros((nf * 3, 3))
    for s, e in zip(starts, ends):
        fs = cf[s:e]
        U = un[fs]
        W = fn[fs]
        mask = (U @ U.T) >= cosc
        N = mask @ W
        cn[order[s:e]] = N
    ln = np.linalg.norm(cn, axis=1); ln[ln == 0] = 1
    cn /= ln[:, None]
    qn = np.round(cn * 127).astype(np.int64)
    key = np.c_[corner_v, qn]
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    inv = inv.reshape(-1)
    return v[corner_v[first]], cn[first], inv.reshape(-1, 3)


# ---------- GLB writer ----------
class Glb:
    def __init__(self):
        self.bin = bytearray(); self.views = []; self.acc = []; self.meshes = []; self.mats = []; self.nodes = []
        self.matkey = {}

    def _view(self, data, target, stride=None):
        while len(self.bin) % 4:
            self.bin.append(0)
        v = {'buffer': 0, 'byteOffset': len(self.bin), 'byteLength': len(data), 'target': target}
        if stride:
            v['byteStride'] = stride
        self.bin += data; self.views.append(v)
        return len(self.views) - 1

    def material(self, rgba, name=None):
        k = tuple(round(c, 3) for c in rgba)
        if k not in self.matkey:
            lum = sum(k[:3]) / 3
            self.mats.append({'name': name or 'm%d' % len(self.mats),
                              'pbrMetallicRoughness': {'baseColorFactor': list(k), 'metallicFactor': 0.25 if 0.45 < lum < 0.8 else 0.05,
                                                       'roughnessFactor': 0.55 if lum > 0.4 else 0.7}})
            self.matkey[k] = len(self.mats) - 1
        return self.matkey[k]

    def mesh(self, name, parts):
        """parts: list of (rgba, pos, nrm, idx)"""
        prims = []
        for rgba, pos, nrm, idx in parts:
            p32 = pos.astype('<f4')
            pv = self._view(p32.tobytes(), 34962)
            self.acc.append({'bufferView': pv, 'componentType': 5126, 'count': len(p32), 'type': 'VEC3',
                             'min': [float(x) for x in p32.min(0)], 'max': [float(x) for x in p32.max(0)]})
            pa = len(self.acc) - 1
            n8 = np.zeros((len(nrm), 4), dtype=np.int8); n8[:, :3] = np.clip(np.round(nrm * 127), -127, 127)
            nv = self._view(n8.tobytes(), 34962, 4)
            self.acc.append({'bufferView': nv, 'componentType': 5120, 'normalized': True, 'count': len(n8), 'type': 'VEC3'})
            na = len(self.acc) - 1
            if len(p32) < 65535:
                ib, ct = idx.astype('<u2').tobytes(), 5123
            else:
                ib, ct = idx.astype('<u4').tobytes(), 5125
            iv = self._view(ib, 34963)
            self.acc.append({'bufferView': iv, 'componentType': ct, 'count': int(idx.size), 'type': 'SCALAR'})
            prims.append({'attributes': {'POSITION': pa, 'NORMAL': na}, 'indices': len(self.acc) - 1, 'material': self.material(rgba)})
        self.meshes.append({'name': name, 'primitives': prims})
        return len(self.meshes) - 1

    def node(self, name=None, t=None, R=None, s=None, mesh=None, extras=None):
        n = {}
        if name: n['name'] = name
        if t is not None and any(abs(x) > 1e-12 for x in t): n['translation'] = [round(float(x), 7) for x in t]
        if R is not None:
            qd = mat_quat(R)
            if abs(qd[3]) < 0.9999999: n['rotation'] = qd
        if s is not None and any(abs(x - 1) > 1e-9 for x in s): n['scale'] = [float(x) for x in s]
        if mesh is not None: n['mesh'] = mesh
        if extras: n['extras'] = extras
        self.nodes.append(n)
        return len(self.nodes) - 1

    def child(self, parent, c):
        self.nodes[parent].setdefault('children', []).append(c)

    def write(self, path, root, extras=None):
        while len(self.bin) % 4:
            self.bin.append(0)
        g = {'asset': {'version': '2.0', 'generator': 'robot-lab build_models.py'}, 'extensionsUsed': ['KHR_mesh_quantization'],
             'extensionsRequired': ['KHR_mesh_quantization'], 'scene': 0, 'scenes': [{'nodes': [root]}], 'nodes': self.nodes,
             'meshes': self.meshes, 'materials': self.mats, 'accessors': self.acc, 'bufferViews': self.views,
             'buffers': [{'byteLength': len(self.bin)}]}
        if extras:
            g['asset']['extras'] = extras
        js = json.dumps(g, separators=(',', ':')).encode()
        js += b' ' * (-len(js) % 4)
        with open(path, 'wb') as f:
            f.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(self.bin)))
            f.write(struct.pack('<II', len(js), 0x4E4F534A)); f.write(js)
            f.write(struct.pack('<II', len(self.bin), 0x004E4942)); f.write(bytes(self.bin))
        return os.path.getsize(path)


# ---------- robot model ----------
class Robot:
    """links: {name: [visual]}, visual = dict(file, xyz, rpy, scale, rgba|None); joints: list of dicts."""
    def __init__(self, name):
        self.name = name; self.links = {}; self.joints = []

    def build(self, path, resolve, budget, skip_links=(), extras=None):
        g = Glb(); cache = {}; stats = {'tris_in': 0, 'tris_out': 0}

        def mesh_for(file, rgba):
            key = (file, tuple(rgba) if rgba else None)
            if key in cache:
                return cache[key]
            p = resolve(file)
            groups = load_dae(p) if p.endswith('.dae') else [(rgba or [0.7, 0.7, 0.7, 1], load_stl(p))]
            merged = {}
            for col, soup in groups:
                merged.setdefault(tuple(round(c, 3) for c in col), []).append(soup)
            parts = []
            for col, soups in merged.items():
                soup = np.concatenate(soups)
                v, f = weld(soup)
                stats['tris_in'] += len(f)
                v, f = simplify(v, f, max(300 if BS == 1 else 60, budget(file, len(f)) * BS))
                pos, nrm, idx = creased(v, f)
                stats['tris_out'] += len(idx)
                parts.append((list(col), pos, nrm, idx))
            cache[key] = g.mesh(os.path.basename(file), parts), parts
            return cache[key]

        children = {}
        for j in self.joints:
            children.setdefault(j['parent'], []).append(j)
        roots = [l for l in self.links if l not in {j['child'] for j in self.joints}]
        lo = np.full(3, np.inf); hi = np.full(3, -np.inf)

        def add_link(link, parent_node, M):
            nonlocal lo, hi
            if link in skip_links:
                return
            for v in self.links.get(link, []):
                mi, parts = mesh_for(v['file'], v.get('rgba'))
                R = rpy_mat(*v['rpy']); s = np.array(v['scale'])
                g.child(parent_node, g.node(None, v['xyz'], R, s, mesh=mi))
                A = M[:3, :3] @ R @ np.diag(s); b = M[:3, :3] @ np.array(v['xyz']) + M[:3, 3]
                for _, pos, _, _ in parts:
                    w = pos @ A.T + b
                    lo = np.minimum(lo, w.min(0)); hi = np.maximum(hi, w.max(0))
            for j in children.get(link, []):
                if j['child'] in skip_links:
                    continue
                R = rpy_mat(*j['rpy'])
                o = g.node(None, j['xyz'], R)
                g.child(parent_node, o)
                ex = None
                if j['type'] in ('revolute', 'continuous', 'prismatic'):
                    ex = {'type': j['type'], 'axis': j['axis'], 'lower': j.get('lower'), 'upper': j.get('upper')}
                    if j.get('mimic'):
                        ex['mimic'] = j['mimic']
                jn = g.node(j['name'] if ex else None, extras=ex)
                g.child(o, jn)
                T = np.eye(4); T[:3, :3] = R; T[:3, 3] = j['xyz']
                add_link(j['child'], jn, M @ T)

        # URDF is Z-up, glTF is Y-up: rotate -90 deg about X at the root
        root = g.node(self.name, None, rpy_mat(-math.pi / 2, 0, 0))
        for r in roots:
            add_link(r, root, np.eye(4))
        ex = dict(extras or {})
        ex['bbox_urdf_m'] = {'min': [round(float(x), 4) for x in lo], 'max': [round(float(x), 4) for x in hi]}
        size = g.write(path, root, ex)
        print(f"{self.name}: {stats['tris_in']} -> {stats['tris_out']} tris, {size/1e6:.2f} MB, bbox(x,y,z m) {ex['bbox_urdf_m']}")
        return ex


def from_urdf(path, name):
    r = ET.parse(path).getroot()
    rb = Robot(name)
    gm = {}
    for m in r.findall('material'):
        c = m.find('color')
        if c is not None:
            gm[m.get('name')] = fl(c.get('rgba'), 4)
    for l in r.findall('link'):
        vs = []
        for v in l.findall('visual'):
            me = v.find('geometry/mesh')
            if me is None:
                continue
            o = v.find('origin'); mat = v.find('material'); rgba = None
            if mat is not None:
                c = mat.find('color')
                rgba = fl(c.get('rgba'), 4) if c is not None else gm.get(mat.get('name'))
            vs.append({'file': me.get('filename'), 'xyz': fl(o.get('xyz') if o is not None else None), 'rpy': fl(o.get('rpy') if o is not None else None),
                       'scale': fl(me.get('scale'), 3, 1.0), 'rgba': rgba})
        rb.links[l.get('name')] = vs
    for j in r.findall('joint'):
        o = j.find('origin'); a = j.find('axis'); lim = j.find('limit'); mi = j.find('mimic')
        d = {'name': j.get('name'), 'type': j.get('type'), 'parent': j.find('parent').get('link'), 'child': j.find('child').get('link'),
             'xyz': fl(o.get('xyz') if o is not None else None), 'rpy': fl(o.get('rpy') if o is not None else None),
             'axis': fl(a.get('xyz')) if a is not None else [1, 0, 0]}
        if lim is not None and j.get('type') != 'continuous':
            d['lower'] = float(lim.get('lower', 0)); d['upper'] = float(lim.get('upper', 0))
        if mi is not None:
            d['mimic'] = {'joint': mi.get('joint'), 'multiplier': float(mi.get('multiplier', 1)), 'offset': float(mi.get('offset', 0))}
        rb.joints.append(d)
    return rb


os.makedirs(OUT, exist_ok=True)
info = {}

# --- SO-101 ---
so = from_urdf(f'{SRC}/so101/Simulation/SO101/so101_new_calib.urdf', 'so101')
for vs in so.links.values():
    for v in vs:
        if v['rgba'] and v['rgba'][0] < 0.2:
            v['rgba'] = [0.13, 0.13, 0.14, 1]
info['so101'] = so.build(f'{OUT}/so101.glb', lambda f: f'{SRC}/so101/Simulation/SO101/{f}', lambda f, n: n * 0.22)

# --- xArm7 (values from xarm_description: xarm7.urdf.xacro + xarm7_default_kinematics.yaml) ---
xa = Robot('xarm7')
kin = [(0, 0, 0.267, 0), (0, 0, 0, -1.5708), (0, -0.293, 0, 1.5708), (0.0525, 0, 0, 1.5708), (0.0775, -0.3425, 0, 1.5708), (0, 0, 0, 1.5708), (0.076, 0.097, 0, -1.5708)]
lim = [(-6.2832, 6.2832), (-2.059, 2.0944), (-6.2832, 6.2832), (-0.19198, 3.927), (-6.2832, 6.2832), (-1.69297, 3.14159), (-6.2832, 6.2832)]
white, silver = [0.93, 0.93, 0.93, 1], [0.753, 0.753, 0.753, 1]
xa.links['link_base'] = [{'file': 'link_base.stl', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0], 'scale': [1, 1, 1], 'rgba': white}]
for i, (k, l) in enumerate(zip(kin, lim), 1):
    xa.links[f'link{i}'] = [{'file': f'link{i}.stl', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0], 'scale': [1, 1, 1], 'rgba': silver if i == 7 else white}]
    xa.joints.append({'name': f'joint{i}', 'type': 'revolute', 'parent': 'link_base' if i == 1 else f'link{i-1}', 'child': f'link{i}',
                      'xyz': list(k[:3]), 'rpy': [k[3], 0, 0], 'axis': [0, 0, 1], 'lower': l[0], 'upper': l[1]})
info['xarm7'] = xa.build(f'{OUT}/xarm7.glb', lambda f: f'{SRC}/xarm7/xarm_description/meshes/xarm7/visual/{f}', lambda f, n: n * 0.75)

# --- OpenArm 2.0 (bimanual, pinch gripper; camera mount omitted) ---
oa = from_urdf(f'{SRC}/openarm/assets/robot/openarm_v2.0/urdf/example/v2.urdf', 'openarm2')
skip = {l for l in oa.links if 'zed' in l or 'camera' in l}
info['openarm2'] = oa.build(f'{OUT}/openarm2.glb', lambda f: f.replace('package://openarm_description', f'{SRC}/openarm'), lambda f, n: n * 0.8, skip_links=skip)

json.dump(info, open(f'{OUT}/info.json', 'w'), indent=1)
