
import sys
sys.path.append('.')
import subprocess
import os

if __name__ == "__main__":
    auto_path = os.path.join(os.path.dirname(__file__), 'frame', 'auto.pyc')
    subprocess.run([sys.executable, auto_path] + sys.argv[1:])
