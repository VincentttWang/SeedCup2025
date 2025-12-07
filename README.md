# ? SeedCup 2025 智能桥梁生成系统

> **队伍名称**：炮灰老人组  
> **赛事**：SeedCup 2025 物理造桥智能算法竞赛  
> **方向**：初赛（逆向工程轻量化设计）& 复赛（自动化通用求解）

---

## ? 背景与挑战

### 赛题概述

SeedCup 2025 物理造桥竞赛是一个**离散优化 + 物理仿真**的综合竞赛：

- ? **输入**：未知地形地图，包含起点、终点、地形障碍
- ?? **目标**：构建**最小成本**的桥梁结构，使 4 类车辆（2 辆 Car、1 辆 Bus、1 辆 Truck）100% 通过
- ?? **约束**：物理仿真 30 秒内完成，结构必须在 29 秒时保持完整
- ? **评分**：`Score = 1000 × (4 - 失败车数) × (100000 - 杆件成本)`

### 核心挑战

| 挑战 | 初赛 | 复赛 |
|------|------|------|
| **地形复杂度** | 已知关卡 | 未知黑盒地形 |
| **杆件数量** | 需极限压缩（<40 根） | 需自动化可靠求解 |
| **时间压力** | 手工微调 | 算法秒速求解 |
| **稳定性** | 单点优化 | 通用鲁棒性 |

---

## ? 项目亮点速览

```
┌─────────────────────────────────────────────────────────────────┐
│  ? 初赛：极简设计 31 根杆件 + 可视化逆向工程工具链           │
│  ? 遗传算法：微种群进化 + 多变异策略 + 自动评估闭环          │
│  ? 工程创新：.level → model.py 自动代码生成 + 差分分析       │
│  ? 复赛：通用求解器 + 混合支撑 + 自适应地形感知              │
│  ?? 并行框架：多进程并发 + 批量仿真 + 窗口自动管理            │
└─────────────────────────────────────────────────────────────────┘
```

---

## ? 项目简介

本项目是一套为 **SeedCup 2025 物理造桥竞赛** 设计的综合解决方案，核心目标是在保证车辆完全通行的前提下，**最小化桥梁成本**（即最少杆件数量）。

### ? 核心亮点

| 特性 | 描述 |
|------|------|
| **? 逆向工程工具链** | 可视化编辑 → 自动化代码生成，"所见即所得" |
| **?? 混合结构生成** | 悬索优先 + 自适应桁架，兼顾成本与稳定性 |
| **? 遗传算法优化** | 微种群进化策略，在极限杆件数下寻找突破 |
| **? 并行仿真框架** | 多进程并发，支持批量关卡高效测试 |
| **? 通用求解器** | 自适应地形感知，支持深渊、悬崖、多层起点 |

---

## ? 项目结构

```
SeedCup2025/
├── README.md                          # 本文件
├── doc/                               # 技术文档
│   ├── 初赛技术报告.md               # 初赛设计与遗传算法探索
│   ├── 复赛技术报告.md               # 复赛通用求解器架构
│   └── SeedCup2025复赛规则.md        # 赛题规则说明
│
├── frame/                             # 核心算法模块
│   ├── bridge_solver.py               # 【初赛】地形分析 & 结构生成
│   ├── model.py                       # 主入口：调用 BridgeSolver
│   ├── solve*.py                      # 关卡特化求解器（初赛手工设计）
│   ├── api.pyc & auto.pyc             # 编译后的底层模拟器接口
│   └── level0.map, level5.map, ...    # 地图数据文件
│
├── utils/                             # 工具集
│   ├── building/                      # 构建相关工具
│   └── level/                         # 关卡处理工具
│
├── release/                           # 赛事提交版本
│   ├── 初赛/
│   │   ├── model.py                   # 初赛最终提交模型（31根杆件）
│   │   └── bridge_solver.py
│   └── 复赛/
│       ├── model.py                   # 复赛通用求解模型
│       └── bridge_solver.py           # 【复赛】增强版求解器
│
├── files/                             # 关卡文件
│   ├── level0.level, level5.level...  # 初赛手工编辑关卡
│   ├── solve0.level, solve5.level...  # 初赛解决方案
│   └── dian.level                     # 基准空地图
│
├── main.py                            # UI 模式（可视化仿真）
├── main_api.py                        # API 模式（静默求解）
├── main_auto.py                       # 自动模式（30s 自动仿真）
│
├── level2map.py                       # .level → .map 转换工具
├── level2model.py                     # .level 逆向工程提取工具
├── level2solve.py                     # 关卡差分分析工具
│
├── run_parallel.py                    # 【并行框架】多关卡并发仿真
│
└── logs/                              # 输出日志（仿真结果、评分）
```

---

## ? 快速开始

### 环境要求

- **Python** 3.11
- **Pygame** 2.7
- **系统** Windows / Linux / macOS

### 基本用法

#### 1?? **UI 可视化模式**（用于调试与设计）

```bash
python main.py
```

启动 Pygame 可视化窗口，实时观察桥梁结构与物理仿真。

#### 2?? **自动仿真模式**（用于评估与测试）

```bash
python main_auto.py --level files/level0.level
```

运行 30 秒自动仿真，在 `logs/` 目录生成评分结果（JSON 格式）。

#### 3?? **API 求解模式**（用于批量处理）

```bash
python main_api.py --level files/level0.level
```

无窗口模式，直接调用 `model()` 生成桥梁结构。

#### 4?? **并行仿真**（用于多关卡测试）

```bash
python run_parallel.py
```

同时启动多个仿真进程（可配置关卡列表），支持 `--nosim` 和 `--sim` 参数。

---

## ?? 算法核心

### 【初赛】基于逆向工程的轻量化设计

#### 设计理念

初赛采用**物理启发式逆向工程**策略：

1. **可视化编辑**：在游戏编辑器中直观设计桥梁
2. **自动代码生成**：通过 `level2model.py` 的差分算法自动提取结构
3. **手工优化**：利用遗传算法进一步探索极限设计空间

#### 核心成果

| 指标 | 数值 | 备注 |
|------|------|------|
| **总杆件数** | **31 根** | 极低成本设计 |
| Road 材料 | 9 根 | 必要的路面 |
| Wood 材料 | 22 根 | 高性价比支撑 |
| 结构形态 | 双岛式 | 左高右低，中间断开（利用重力势能） |

#### 关键创新：跳跃式设计

```
[起点高台 Y=19]  ──→  [自由落体]  ──→  [缓冲坡 Y=3]
     急速助跑        穿越中间区域        柔性着陆
   利用势能加速                      反冲上升到终点
```

### 【复赛】通用自适应求解器（BridgeSolver）

#### 系统架构

```
Map Data (JSON)
        ↓
[环境分类]  ← 识别起点、终点、障碍、支撑点
        ↓
[路径规划]  ← 避障、平滑、多层支持
        ↓
[结构生成]  ← 悬索优先 + 自适应桁架
        ↓
[剪枝优化]  ← 删除冗余杆件
        ↓
Sticks Output (Dict)
```

#### 混合支撑策略

**第一阶段：悬索优先**
- 检测桥面上方的"天花板节点"（Ceiling Nodes）
- 若可用且距离 < 12.0，生成钢索（Steel）悬挂支撑
- 优势：大幅减少下方材料，成本更低

**第二阶段：自适应桁架**
- 对未被悬索覆盖的区域生成倒三角桁架
- 动态计算高度（深渊处更厚，近地处更扁）
- 自动锚定：搜索 8.0 范围内的固定点连接

#### 关键代码片段

```python
def _grow_supports(self, deck_points, ceiling_nodes, ground_clusters, is_main=False):
    # 悬索优先逻辑
    suspended_indices = set()
    for i, pt in enumerate(deck_points):
        best_ceil = self._find_nearest_ceiling(pt, ceiling_nodes)
        if best_ceil and dist(pt, best_ceil) < 12.0:
            self._add_stick(pt, best_ceil, 'steel')
            suspended_indices.add(i)
    
    # 自适应桁架
    for i in range(len(deck_points) - 1):
        if i in suspended_indices:
            truss_height = 1.0  # 悬挂区域最小化
        else:
            truss_height = max(1.0, 5.0 - dist_to_ground/15.0)
        # 生成桁架节点与连杆...
```

---

## ? 遗传算法探索（初赛创新）

### 算法概述

虽然最终提交了手工优化方案，但我们还探索了 **Micro-GA（微种群遗传算法）**：

- **种子初始化**：以手工设计的 31 根杆件方案为"夏娃"
- **微种群**：6-10 个个体，专注局部搜索
- **变异策略**：
  - 坐标微扰（70%）：±2 Grid 抖动，寻找更优应力分布
  - 材料降级（20%）：Road → Wood 替换，降低成本
  - 极限减重（10%）：随机删除非关键杆件
- **评估方式**：直接调用 `main_auto.py`，通过 stdout 解析得分

### 实验结论

? **验证可行性**：证明 AI 辅助设计在结构优化中的潜力  
?? **成本权衡**：每次评估 30 秒 → 完整收敛需数小时，综合考虑后采用手工方案  
? **技术积累**：为复赛更复杂场景提供算法基础

---

## ? 核心工具使用

### 1. 逆向工程工具：`level2model.py`

将游戏内编辑的 `.level` 文件自动转换为 `model.py` 代码。

```bash
python level2model.py files/jump.level files/dian.level frame/model.py
```

**原理**：
- 读取两个关卡文件（有桥 vs. 无桥）
- 计算坐标差分（坐标/40 后网格化）
- 提取唯一杆件签名 `(p0, p1, material)`
- 自动生成规范 Python 代码

### 2. 关卡分析工具：`level2solve.py`

对比原始关卡与解决方案，提取新增杆件。

```bash
python level2solve.py files/level5.level files/solve5.level frame/solve5.py
```

### 3. 格式转换工具：`level2map.py`

将 `.level` 文件转换为 `.map` 格式（复赛使用）。

```bash
python level2map.py files/level0.level frame/level0.map
```

---

## ? 性能指标

### 初赛成果

| 关卡 | 杆件数 | 材料组成 | 通过状态 |
|------|--------|--------|--------|
| level0 | 36 | R:9 W:27 | ? 100% |
| level5 | 34 | R:8 W:26 | ? 100% |
| level8 | 31 | R:9 W:22 | ? 100% |
| level666 | 28 | R:6 W:22 | ? 100% |

### 复赛能力

? **多层起点支持**：识别不同高度的发车平台  
? **动态避障**：递归检测并规避地形障碍  
? **自适应成本**：根据地形深度动态调整结构厚度  
? **稳健性**：处理极端地形（深渊、悬崖、多角度路面）

---

## ? 核心文件说明

### `frame/bridge_solver.py`（初赛版，411 行）

**主类**：`BridgeSolver`

**核心方法**：
- `_analyze_map()` - 地形特征识别（起点、终点、障碍、支撑点）
- `_build_path()` - 路径规划与避障
- `_emit_segment()` - 杆件生成与去重
- `solve()` - 主求解流程，返回杆件字典

**特点**：
- 启发式规则，快速求解
- 依赖手工设计与微调
- 高效率，低成本

### `release/复赛/bridge_solver.py`（复赛版，2067 行）

**增强特性**：
- 完整的环境分类系统（天花板、地面支撑聚类）
- 递归避障算法
- 混合支撑生成（悬索 + 桁架）
- 迭代剪枝优化
- 内置关卡缓存与预计算方案

**适用场景**：
- 通用多关卡自动求解
- 复赛难度关卡（level0-level10）
- 完全黑盒地图数据

### `frame/model.py`

**入口函数**：`model(map_data=None, map_path=None) -> dict`

**用途**：
- 接收地图字典或文件路径
- 调用 BridgeSolver 求解
- 返回 `{'sticks': [...]}` 格式

**示例**：
```python
def model(map_data: dict = None, map_path: str = None) -> dict:
    if map_data is None:
        map_data = _load_map_for_testing(map_path)
    
    solver = BridgeSolver(map_data)
    return solver.solve()
```

### `run_parallel.py`

**功能**：多进程并发仿真

**配置**：
```python
levels_to_run = [
    'files/level0.level',
    'files/level5.level',
    'files/level8.level',
    'files/level666.level',
]
```

**使用**：
```bash
python run_parallel.py --nosim     # 只生成不仿真
python run_parallel.py --sim       # 完整仿真
```

---

## ? 技术深度

### 几何算法

- **坐标网格化**：`coord / 40` 取整，统一坐标系
- **距离度量**：欧氏距离 + 范围阈值搜索
- **碰撞检测**：线段相交判定（参数方程法）
- **平滑曲线**：分段线性插值 + Bezier 可选

### 物理启发式

- **悬索模型**：重力负荷 → 张力分解
- **桁架模型**：倒三角几何 → 剪力与弯矩承载
- **剪枝策略**：识别"只连接一个非固定点"的冗余杆件

### 优化策略

- **贪心选择**：距离最近、成本最低的支撑点
- **精英保留**：遗传算法中保留历代最优解
- **参数自适应**：根据地形深度动态调整结构参数

### 实战案例：初赛关卡优化演进

**关卡 level0** 的设计演进：

```
第 1 版：直线路面 → 40 根杆件 ? 成本过高
   └─ 问题：过度设计，未利用地形

第 2 版：利用悬崖支撑 → 36 根杆件 ? 改善
   └─ 方案：添加锚点连接，减少下方支撑

第 3 版：优化桁架几何 → 34 根杆件 ? 优化
   └─ 方案：倒三角密集化，提高刚度

最终版：手工微调 + GA 变异 → 36 根杆件 ? 稳定
   └─ 方案：平衡成本与稳定性
```

**关卡 level666**（难关）的反向思维：

```
传统方案：连接起点 → 终点（跨度大，杆件多）
         需要 50+ 根杆件，难以稳定

炮灰方案：起点 ↓↓↓（自由落体）↓↓↓ 终点
         利用重力势能，仅需 28 根杆件！
         ? 更轻  ? 更快  ? 更便宜
```

---

## ? 调试与测试

### 启用调试模式

编辑 `frame/model.py` 中的 `MAP_FILE_PATH`，指定测试地图：

```python
MAP_FILE_PATH = 'maps/level5.map'
```

然后运行本地测试：
```bash
python frame/model.py
```

输出示例：
```
--- 本地测试模式 ---
正在加载地图: maps/level5.map
调用 model() 函数生成桥梁...
  -> 发现可利用路面: Node 10 - 15 (Angle: 5.2°)
  -> 识别到 8 个支撑点用于桁架连接
...
总共生成 34 根杆件。
测试结果已写入 frame/model_test_output.py
```

### 查看评分日志

仿真完成后，检查 `logs/` 目录：

```bash
ls -l logs/
# 输出：result_2025-12-08_143022.json
cat logs/result_2025-12-08_143022.json
```

JSON 格式：
```json
{
  "score": 8500,
  "success": true,
  "time_elapsed": 30.0,
  "vehicles": [
    {"type": "car", "passed": true, "distance": 2048},
    {"type": "car", "passed": true, "distance": 2048},
    {"type": "bus", "passed": true, "distance": 2048},
    {"type": "truck", "passed": true, "distance": 2048}
  ]
}
```

### 参数调优指南

**位置**：`frame/bridge_solver.py` 顶部（复赛版在 `release/复赛/bridge_solver.py`）

```python
MAX_BEAM_LENGTH = 8.0              # 单根杆件最大长度，超过则分段
OPTIMAL_TRUSS_HEIGHT = 5.0         # 理想桁架层高，深渊处使用
WALKABLE_ANGLE_THRESHOLD = 35.0    # 路面最大倾角（度），超过视为陡峭
GRID_STEP = 2.0                    # 路径规划网格精度，越小精度越高但计算量大

# 复赛特有参数
SEARCH_RADIUS = 8.0                # 搜索支撑点的半径
MAX_NODE_DEGREE = 6                # 单个节点最多连接数（防止过度连接）
```

**调优建议**：
- 如果结构不稳定 → 增大 `OPTIMAL_TRUSS_HEIGHT`
- 如果成本过高 → 减小 `MAX_BEAM_LENGTH` 或 `SEARCH_RADIUS`
- 如果路面抖动 → 增大 `WALKABLE_ANGLE_THRESHOLD`
- 如果计算过慢 → 增大 `GRID_STEP` 或 `SEARCH_RADIUS`

---

## ?? 性能优化建议

### 1. 内存优化
- 使用 `seen_edges` 集合去重，避免重复杆件
- 预计算节点度数，避免重复遍历

### 2. 计算优化
- 采用"笛卡尔积"式的近邻搜索，而不是全量遍历
- 利用网格数据结构加速空间查询（可选）

### 3. 稳定性保证
- 迭代剪枝过程中保留至少一层下支撑
- 对关键节点（起点、终点）禁用删除

---

## ? 常见问题排查

| 问题 | 症状 | 解决方案 |
|------|------|--------|
| **地图加载失败** | `FileNotFoundError: level*.map not found` | 确保 `.map` 文件在 `frame/` 目录，或使用 `level2map.py` 转换 |
| **杆件生成为空** | `生成 0 根杆件` | 检查 `_analyze_map()` 是否正确识别起点与终点 |
| **车辆掉落** | 物理仿真中车辆从桥上掉落 | 增加桁架高度、添加更多支撑点、或调整路面倾角 |
| **仿真超时** | 30 秒内未完成评估 | 减少杆件数量、简化结构几何 |
| **并行窗口错乱** | 多个窗口重叠或显示混乱 | 修改 `run_parallel.py` 中的启动延迟 `time.sleep(0.5)` |

---

## ? 赛事回顾

### 初赛：逆向工程创新

**挑战**：手工编码坐标繁琐且易出错

**解决方案**：
1. 开发可视化逆向工具链
2. 设计"跳跃式"轻量化结构（31 根杆件）
3. 探索遗传算法优化空间

**成果**：极简设计 + 工程化工具 + 算法验证的综合方案

### 复赛：通用求解器升级

**挑战**：未知地形、多起点、黑盒求解

**解决方案**：
1. 完整的地形感知与分类系统
2. 混合支撑策略（悬索优先）
3. 自适应参数与剪枝优化

**成果**：支持 level0-level10 的通用求解框架

---

## ? 快速命令参考

| 任务 | 命令 | 说明 |
|------|------|------|
| **UI 仿真** | `python main.py` | 启动可视化界面，实时预览 |
| **自动评估** | `python main_auto.py --level files/level0.level` | 自动 30s 仿真，输出评分 |
| **批量测试** | `python run_parallel.py` | 并发多关卡（修改配置列表） |
| **代码生成** | `python level2model.py files/jump.level files/dian.level frame/model.py` | 逆向提取桥梁结构 |
| **格式转换** | `python level2map.py files/level0.level frame/level0.map` | .level → .map 转换 |
| **查看日志** | `cat logs/result_*.json` | 检查最新评分结果 |
| **本地测试** | `python frame/model.py` | 调试 model.py（修改 MAP_FILE_PATH） |
| **关卡分析** | `python level2solve.py files/level5.level files/solve5.level frame/solve5.py` | 提取关卡新增杆件 |

---

## ? 技术文档

详细的算法设计、代码注释、性能分析见：

- ? **初赛技术报告**：`doc/初赛技术报告：基于逆向工程的轻量化跳跃桥设计.md`
  - 场景分析、工作流架构、结构设计详解、遗传算法探索

- ? **复赛技术报告**：`doc/复赛技术报告.md`
  - 系统概述、分层生成策略、并行框架、关键代码展示

- ? **赛题规则**：`doc/SeedCup2025复赛规则.md`
  - 规则说明、地图格式、评分机制

### 从初赛迁移到复赛

如需切换求解器：

```python
# frame/model.py - 修改导入
from bridge_solver import BridgeSolver  # 初赛版本

# 或者
import sys
sys.path.insert(0, 'release/复赛')
from bridge_solver import BridgeSolver  # 复赛版本
```

**何时使用哪个版本**：
- 初赛版本：已知特定关卡，需要极限成本优化
- 复赛版本：未知地形，需要通用可靠求解

---

## ? 贡献与维护

本项目为 **SeedCup 2025 竞赛队伍作品**。

### 如何扩展

1. **添加新关卡**：
   - 将 `.level` 文件放入 `files/`
   - 在 `run_parallel.py` 中添加关卡列表
   - 运行仿真并查看日志

2. **调整算法参数**：
   - `frame/bridge_solver.py` 顶部的配置参数
   - `MAX_BEAM_LENGTH`, `OPTIMAL_TRUSS_HEIGHT`, `WALKABLE_ANGLE_THRESHOLD`

3. **集成新求解器**：
   - 在 `frame/` 中创建新的求解器类
   - 修改 `model.py` 中的求解器调用

---

## ? 联系方式

如有技术问题或改进建议，欢迎通过以下方式联系：

- **GitHub Issue**：提交代码 Bug 或功能建议
- **技术文档**：详见 `doc/` 目录下的完整报告

