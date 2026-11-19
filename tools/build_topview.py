"""Top-view robot glyphs for the floor plan (the TOP table in floorplan.html).

Each robot is put in one pose, every link's mesh is projected onto the floor and the convex hull of each
(link, light/dark colour) group is kept as an SVG path. Units are cm, the robot's front is −y, the centre of its base is the origin.
Prints the `const TOP = {...}` block to paste into floorplan.html.

usage: python3 tools/build_topview.py <folder with the downloaded sources>   (same folder layout as build_models.py)
"""
import json, math, os, sys
import numpy as np

SRC = sys.argv[1]
HERE = os.path.dirname(os.path.abspath(__file__))
code = open(os.path.join(HERE, 'build_models.py'), encoding='utf-8').read().split("os.makedirs(OUT")[0]  # loaders and URDF reader only
sys.argv = [sys.argv[0], SRC, '.']; ns = {}; exec(code, ns)
Robot, from_urdf, rpy_mat, load_stl, load_dae = ns['Robot'], ns['from_urdf'], ns['rpy_mat'], ns['load_stl'], ns['load_dae']

# pose drawn on the plan, and how far to slide the drawing so the base (not the URDF origin) is centred on the item
POSES = {
    'openarm2': ({'openarm_left_joint4': 1.57, 'openarm_right_joint4': 1.57}, -3.0),
    'xarm7': ({'joint2': -0.35, 'joint4': 1.2, 'joint6': 1.55}, 0.0),
    'so101': ({'shoulder_lift': -0.6, 'elbow_flex': 0.9, 'wrist_flex': 0.7}, 1.7),
}

def axis_rot(a, q):
    a = np.array(a, float); a /= np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K

_cache = {}
def groups_of(path, rgba):
    key = (path, tuple(rgba) if rgba else None)
    if key not in _cache:
        g = load_dae(path) if path.endswith('.dae') else [(rgba or [0.7, 0.7, 0.7, 1], load_stl(path))]
        _cache[key] = [(list(c)[:3], np.asarray(p, float).reshape(-1, 3)) for c, p in g]
    return _cache[key]

def fk(rb, resolve, q, skip=()):
    ch = {}
    for j in rb.joints: ch.setdefault(j['parent'], []).append(j)
    roots = [l for l in rb.links if l not in {j['child'] for j in rb.joints}]
    out = []
    def rec(link, M):
        if link in skip: return
        for v in rb.links.get(link, []):
            A = M[:3, :3] @ rpy_mat(*v['rpy']) @ np.diag(v['scale']); b = M[:3, :3] @ np.array(v['xyz']) + M[:3, 3]
            for c, p in groups_of(resolve(v['file']), v.get('rgba')): out.append((link, c, p @ A.T + b))
        for j in ch.get(link, []):
            T = np.eye(4); T[:3, :3] = rpy_mat(*j['rpy']); T[:3, 3] = j['xyz']
            val = q.get(j['name'], 0.0)
            if j.get('mimic'): val = q.get(j['mimic']['joint'], 0.0) * j['mimic']['multiplier'] + j['mimic']['offset']
            if 'lower' in j: val = min(max(val, j['lower']), j['upper'])
            if j['type'] in ('revolute', 'continuous'):
                R = np.eye(4); R[:3, :3] = axis_rot(j['axis'], val); T = T @ R
            elif j['type'] == 'prismatic':
                a = np.array(j['axis'], float); T[:3, 3] = T[:3, 3] + T[:3, :3] @ (a / np.linalg.norm(a) * val)
            rec(j['child'], M @ T)
    for r in roots: rec(r, np.eye(4))
    return out

def hull(P):
    P = np.unique(np.round(P, 4), axis=0); P = P[np.lexsort((P[:, 1], P[:, 0]))]
    if len(P) < 3: return P
    def half(pts):
        h = []
        for p in pts:
            while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (p[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0]) <= 0: h.pop()
            h.append(p)
        return h
    lo, up = half(P), half(P[::-1])
    return np.array(lo[:-1] + up[:-1])

def thin(H, tol, nmax):
    """drop the vertex whose removal loses the least area until every removal would lose more than tol (cm²) and at most nmax remain"""
    H = [tuple(p) for p in H]
    while len(H) > 4:
        n = len(H); best = None
        for i in range(n):
            a, b, c = H[i - 1], H[i], H[(i + 1) % n]
            ar = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2
            if best is None or ar < best[0]: best = (ar, i)
        if best[0] > tol and n <= nmax: break
        H.pop(best[1])
    return np.array(H)

def tone(c):  # 0 light … 1 dark
    return 1 - (0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2])

def glyph(rb, resolve, q, skip=(), tol=0.05, nmax=18, merge=None):
    parts = {}
    for link, c, P in fk(rb, resolve, q, skip):
        key = (merge(link) if merge else link, 'd' if tone(c) > 0.55 else 'l')
        parts.setdefault(key, []).append(P)
    out = []
    for (link, t), Ps in parts.items():
        P = np.concatenate(Ps)
        xy = np.stack([-P[:, 1], -P[:, 0]], 1) * 100  # plan: forward (URDF +x) is up (−y), URDF +y (left) is −x; cm
        H = thin(hull(xy), tol, nmax)
        if len(H) < 3: continue
        area = abs(sum(H[i - 1][0] * H[i][1] - H[i][0] * H[i - 1][1] for i in range(len(H)))) / 2
        if area < 0.6: continue
        out.append((float(P[:, 2].max()), t, H, link))
    out.sort(key=lambda o: o[0])
    return out

def path(H):
    s = 'M' + 'L'.join(f'{x:.1f} {y:.1f}'.replace('-0.0', '0.0') for x, y in H) + 'Z'
    return s.replace('.0 ', ' ').replace('.0L', 'L').replace('.0Z', 'Z')

# ---- robots ----
so = from_urdf(f'{SRC}/so101/Simulation/SO101/so101_new_calib.urdf', 'so101')
for vs in so.links.values():
    for v in vs:
        if v['rgba'] and v['rgba'][0] < 0.2: v['rgba'] = [0.13, 0.13, 0.14, 1]
xa = Robot('xarm7')
kin = [(0, 0, 0.267, 0), (0, 0, 0, -1.5708), (0, -0.293, 0, 1.5708), (0.0525, 0, 0, 1.5708), (0.0775, -0.3425, 0, 1.5708), (0, 0, 0, 1.5708), (0.076, 0.097, 0, -1.5708)]
lim = [(-6.2832, 6.2832), (-2.059, 2.0944), (-6.2832, 6.2832), (-0.19198, 3.927), (-6.2832, 6.2832), (-1.69297, 3.14159), (-6.2832, 6.2832)]
xa.links['link_base'] = [{'file': 'link_base.stl', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0], 'scale': [1, 1, 1], 'rgba': [1, 1, 1, 1]}]
for i, (k, l) in enumerate(zip(kin, lim), 1):
    xa.links[f'link{i}'] = [{'file': f'link{i}.stl', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0], 'scale': [1, 1, 1], 'rgba': [0.7, 0.7, 0.7, 1] if i == 7 else [1, 1, 1, 1]}]
    xa.joints.append({'name': f'joint{i}', 'type': 'revolute', 'parent': 'link_base' if i == 1 else f'link{i-1}', 'child': f'link{i}', 'xyz': list(k[:3]), 'rpy': [k[3], 0, 0], 'axis': [0, 0, 1], 'lower': l[0], 'upper': l[1]})
oa = from_urdf(f'{SRC}/openarm/assets/robot/openarm_v2.0/urdf/example/v2.urdf', 'openarm2')
oskip = {l for l in oa.links if 'zed' in l or 'camera' in l}
R = {
    'so101': (so, lambda f: f'{SRC}/so101/Simulation/SO101/{f}', ()),
    'xarm7': (xa, lambda f: f'{SRC}/xarm7/xarm_description/meshes/xarm7/visual/{f}', ()),
    'openarm2': (oa, lambda f: f.replace('package://openarm_description', f'{SRC}/openarm'), oskip),
}

print('const TOP = {')
for name, (q, dy) in POSES.items():
    rb, res, skip = R[name]
    g = glyph(rb, res, q, skip, tol=0.09, nmax=13)
    allp = np.concatenate([H for _, _, H, _ in g]) + [0, dy]
    box = [round(float(v), 1) for v in (allp[:, 0].min(), allp[:, 1].min(), allp[:, 0].max(), allp[:, 1].max())]
    parts = [t + path(H + [0, dy]) for _, t, H, _ in g]
    print('  %s: { box: %s, parts: %s },' % (name, json.dumps(box), json.dumps(parts, separators=(',', ':'))))
print('};')
