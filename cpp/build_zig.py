"""Build planner._fastgrid with Zig's bundled clang, for Windows machines whose
Visual Studio Build Tools lack the Windows SDK.

    pip install ziglang pybind11
    python cpp/build_zig.py
"""
import subprocess
import sys
import sysconfig
from pathlib import Path

import pybind11

root = Path(__file__).resolve().parent.parent
out = root / "planner" / ("_fastgrid" + sysconfig.get_config_var("EXT_SUFFIX"))
libs = Path(sys.base_prefix) / "libs"
pylib = f"python{sys.version_info.major}{sys.version_info.minor}"
subprocess.run([
    sys.executable, "-m", "ziglang", "c++", "-O2", "-std=c++17", "-shared", "-w",
    "-target", "x86_64-windows-gnu",
    "-I", pybind11.get_include(), "-I", sysconfig.get_paths()["include"],
    str(root / "cpp" / "fastgrid.cpp"),
    "-L", str(libs), f"-l{pylib}", "-o", str(out),
], check=True)
for junk in [*out.parent.glob("*fastgrid*.lib"), *out.parent.glob("*fastgrid*.pdb")]:
    junk.unlink()  # linker by-products; only the .pyd is needed
print(f"built {out}")
