"""
将 .level 文件中的 sticks 信息转换为 model.py 格式
用法：python level2model.py files/jump2.level files/dian.level frame/model.py
"""
import sys
import json
import os

def get_sticks(level_path):
    with open(level_path, 'r', encoding='utf-8') as f:
        level = json.load(f)
    points = level.get('points', [])
    id2xy = {p['id']: [int(round(p['x']/40)), int(round(p['y']/40))] for p in points if 'id' in p and 'x' in p and 'y' in p}
    sticks = []
    for s in level.get('sticks', []):
        if not s.get('prefab', False):
            continue  # 只处理 prefab=true 的桥梁
        p0id = s.get('point0_id')
        p1id = s.get('point1_id')
        mat = s.get('material', 'road')
        if p0id not in id2xy or p1id not in id2xy:
            continue
        p0 = id2xy[p0id]
        p1 = id2xy[p1id]
        sticks.append({
            'p0': p0,
            'p1': p1,
            'material': mat
        })
    return sticks

def level_diff_sticks(jump_path, dian_path, model_path):
    jump_sticks = get_sticks(jump_path)
    dian_sticks = get_sticks(dian_path)
    # 用 tuple 标识唯一性
    def stick_key(s):
        return (tuple(s['p0']), tuple(s['p1']), s['material'])
    dian_keys = set(stick_key(s) for s in dian_sticks)
    unique_sticks = [s for s in jump_sticks if stick_key(s) not in dian_keys]
    # 写入 model.py 格式
    header = '"""\n自动生成的 model.py, 来源于 jump.level 独有 stick\n"""\n\ndef model() -> dict:\n    sticks = [\n'
    for s in unique_sticks:
        header += f"        {{'p0': {s['p0']}, 'p1': {s['p1']}, 'material': '{s['material']}' }},\n"
    header += "    ]\n\n    return {'sticks': sticks}\n"
    with open(model_path, 'w', encoding='utf-8') as f:
        f.write(header)
    print(f'已将 jump.level 独有的桥梁信息写入 {model_path}')

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print('用法: python level2model.py files/jump.level files/dian.level frame/model.py')
    else:
        level_diff_sticks(sys.argv[1], sys.argv[2], sys.argv[3])
