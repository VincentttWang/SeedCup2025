
import sys
sys.path.append('.')
import subprocess
import os

if __name__ == "__main__":
    api_path = os.path.join(os.path.dirname(__file__), 'frame', 'api.pyc')
    subprocess.run([sys.executable, api_path] + sys.argv[1:])
