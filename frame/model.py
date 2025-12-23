"""
主模型文件，调用桥梁生成算法
"""
import json
import sys
import os

# --- 可配置参数 ---
# 在这里指定要读取的地图文件路径
# 例如: 'level8.map' (与 model.py 同目录)
MAP_FILE_PATH = 'frame/level5.map' 

# 尝试从命令行参数中获取 --level 参数来覆盖 MAP_FILE_PATH
# 这样在使用 main_auto.py 或直接运行此脚本时，可以通过 --level 指定地图
if '--level' in sys.argv:
    try:
        idx = sys.argv.index('--level')
        if idx + 1 < len(sys.argv):
            level_arg = sys.argv[idx + 1]
            # 将 files/level5.level 转换为 level5.map
            # 1. 获取文件名（不带后缀）
            basename = os.path.splitext(os.path.basename(level_arg))[0]
            # 2. 构造 map 路径 (假设在当前目录下)
            current_dir = os.path.dirname(os.path.abspath(__file__))
            MAP_FILE_PATH = os.path.join(current_dir, f'{basename}.map')
            print(f"检测到命令行参数，已将 MAP_FILE_PATH 更新为: {MAP_FILE_PATH}")
    except ValueError:
        pass

def _get_map_signature(data):
    """提取地图的关键特征用于比对：所有节点的坐标集合"""
    nodes = data.get('nodes', data.get('points', []))
    # 提取 (x, y, pinned) 元组，忽略 id
    # 使用排序后的列表来比较
    sigs = []
    for n in nodes:
        # 保留一位小数，避免浮点精度问题
        x = round(float(n.get('x', 0)), 1)
        y = round(float(n.get('y', 0)), 1)
        pinned = bool(n.get('pinned', False))
        sigs.append((x, y, pinned))
    return sorted(sigs)


def model(map_data: dict = None, map_path: str = None) -> dict:
    """
    调用 BridgeSolver 生成桥梁结构。
    
    Args:
        map_data (dict, optional): 已加载的地图数据。
        map_path (str, optional): 地图文件路径。当 map_data 为 None 时使用。
        
    Returns:
        dict: 包含 'sticks' 列表的字典，用于物理引擎模拟。
    """

    # 动态导入 BridgeSolver，确保在函数调用时加载
    # 并添加当前目录到 sys.path 以防止导入错误
    import sys
    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if current_dir not in sys.path:
        sys.path.append(current_dir)
    
    from bridge_solver import BridgeSolver, BridgeSolver2

    if map_data is None:
        source_path = map_path or MAP_FILE_PATH
        map_data = _load_map_for_testing(source_path)
        if map_data is None:
            return {'sticks': []}

    solver = BridgeSolver(map_data)
    
    return solver.solve()

# --- 以下为本地测试时使用的代码 ---
def _load_map_for_testing(path: str) -> dict:
    """辅助函数，用于从文件加载地图数据进行测试。"""
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"错误: 地图文件未找到 -> {path}")
        return None
    except json.JSONDecodeError:
        print(f"错误: 地图文件格式不正确 -> {path}")
        return None

if __name__ == '__main__':
    # 这是一个本地测试的例子
    # 实际比赛中，框架会直接调用 model(map_data) 函数
    
    print(f"--- 本地测试模式 ---")
    print(f"正在加载地图: {MAP_FILE_PATH}")
    
    # 1. 调用 model 函数
    print("调用 model() 函数生成桥梁...")
    result_sticks = model(map_path=MAP_FILE_PATH)

    if result_sticks is None:
        sys.exit(1)

    # 2. 打印结果
    print("\n--- 生成的 Sticks 结果 ---")
    sticks = result_sticks.get('sticks') if isinstance(result_sticks, dict) else None
    if sticks:
        for i, stick in enumerate(sticks):
            print(f"  {i+1}: {stick}")
        print(f"\n总共生成 {len(sticks)} 根杆件。")
    else:
        print("未能生成任何杆件。")
    
    # 3. (可选) 将结果写入一个临时的 model_test_output.py 文件，方便检查
    try:
        with open('frame/model_test_output.py', 'w', encoding='utf-8') as f:
            f.write('import json\n\n')
            f.write('def get_sticks():\n')
            f.write('    return ' + json.dumps(result_sticks, indent=4))
        print("\n测试结果已写入 frame/model_test_output.py")
    except Exception as e:
        print(f"\n写入测试文件失败: {e}")
