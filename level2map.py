import sys
import os
import subprocess

def main():
    if len(sys.argv) < 2:
        print("用法: python level2map.py [path of input.level] [name of output.map]")
        return

    input_file = sys.argv[1]

    # 默认输出文件名
    if len(sys.argv) >= 3:
        output_file = sys.argv[2]
    else:
        output_file = os.path.splitext(input_file)[0] + ".map"

    # 找到 api.pyc 的路径（即 frame/api.pyc）
    api_path = os.path.join(os.path.dirname(__file__), "frame", "api.pyc")

    if not os.path.exists(api_path):
        print("错误: 找不到 frame/api.pyc")
        return

    # 调用加密后的 api.pyc 执行转换
    cmd = [
        sys.executable,
        api_path,
        input_file,
        "--output",
        output_file,
    ]

    subprocess.run(cmd)

if __name__ == "__main__":
    main()
