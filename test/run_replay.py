"""Run the actual portable C++ measurement core without ESP32 hardware."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
directory = root / ".pio" / "replay"
directory.mkdir(parents=True, exist_ok=True)
executable = directory / "replay.exe"
subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-pedantic",
                str(root / "test/measurement_replay.cpp"), "-o", str(executable)], check=True)
subprocess.run([str(executable)], check=True)
