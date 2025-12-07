import subprocess
import sys
import time
import os

# 定义要运行的关卡列表
# 你可以根据需要修改这里的文件路径
# 注意：确保对应的 .map 文件存在于 maps/ 目录下
levels_to_run = [
    'files/level0.level',
    'files/level5.level',
    'files/level8.level', 
    'files/level666.level',
]

def run_parallel_simulations():
    processes = []
    print(f"--- 准备并发启动 {len(levels_to_run)} 个仿真任务 ---")

    # 创建 img 目录
    if not os.path.exists('img'):
        os.makedirs('img')

    # 获取传递给子进程的参数 (--nosim 或 --sim)
    extra_args = [arg for arg in sys.argv[1:] if arg in ['--nosim', '--sim']]

    for i, level in enumerate(levels_to_run):
        # 检查文件是否存在
        if not os.path.exists(level):
            print(f"警告: 文件不存在 {level}，跳过。")
            continue

        # 使用 main_auto.py 正常运行
        # 并传递 extra_args
        cmd = [sys.executable, 'main_auto.py', '--level', level] + extra_args
        print(f"正在启动: {level} ...")
        
        # 设置环境变量以控制窗口位置
        try:
            p = subprocess.Popen(cmd)
            processes.append(p)
        except Exception as e:
            print(f"启动失败 {level}: {e}")

        # 稍微错开启动时间
        time.sleep(0.5)

    if not processes:
        print("没有成功启动任何进程。")
        return

    print(f"\n已成功启动 {len(processes)} 个仿真窗口。")
    print("主程序正在运行，按 Ctrl+C 可以强制关闭所有窗口。")

    try:
        # 等待所有子进程结束
        for p in processes:
            p.wait()
    except KeyboardInterrupt:
        print("\n\n检测到中断信号 (Ctrl+C)。")
        print("正在终止所有仿真进程...")
        for p in processes:
            if p.poll() is None: # 如果进程还在运行
                p.terminate()
        print("所有进程已终止。")

if __name__ == "__main__":
    run_parallel_simulations()
