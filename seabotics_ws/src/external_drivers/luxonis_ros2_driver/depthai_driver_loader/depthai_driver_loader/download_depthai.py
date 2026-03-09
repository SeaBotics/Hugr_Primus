import os
import subprocess
from pathlib import Path

REPO_URL = "https://github.com/luxonis/depthai-ros.git"
TARGET_DIR = Path(__file__).resolve().parents[2] / "depthai-ros"
ROS_DISTRO = "Humble"

def main():
    if TARGET_DIR.exists():
        print(f"[depthai_driver_loader] depthai-ros already exists at {TARGET_DIR}")
        return

    print(f"[depthai_driver_loader] Cloning depthai-ros into {TARGET_DIR}")
    subprocess.check_call(["git", "clone", "--branch", ROS_DISTRO, REPO_URL, str(TARGET_DIR)])

if __name__ == "__main__":
    main()
