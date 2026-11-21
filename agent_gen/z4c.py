"""Apply the Z4c transform to a gN design dict: free (undriven) helper wheels, no helper gear train,
helper-axle play closed with bushes, chassis = centred deck plate."""
import json
def strip(d, names):
    d['parts'] = [p for p in d['parts'] if p['name'] not in names]
    d['connections'] = [c for c in d['connections'] if c['child'] not in names and c['parent'] not in names]
    d['gear_meshes'] = [m for m in d['gear_meshes'] if m['gear_a'] not in names and m['gear_b'] not in names]
def z4c(d, name):
    d['name'] = name
    strip(d, {f'{n}{s}' for n in ('h8g', 'hcg', 'hhg', 'h8a', 'h8b', 'hca', 'hcb') for s in 'LR'})
    hha = {q['name']: q for q in d['parts'] if q['name'] in ('hhaL', 'hhaR')}
    for s, sg in (('L', 1), ('R', -1)):
        a = hha['hha' + s]
        for i, yy in enumerate((40.0, 44.0, 58.0)):
            n = f'hfill{s}{i}'
            d['parts'].append({'name': n, 'part': '4265c', 'pos_mm': [a['pos_mm'][0], sg * yy, a['pos_mm'][2]], 'yaw_deg': 0.0, 'spin_deg': 0.0})
            d['connections'].append({'child': n, 'parent': 'hha' + s, 'kind': 'cross'})
    d['chassis'] = 'deck'
    return d
