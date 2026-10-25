"""Builds the optional C++ extension planner._fastgrid (needs a C++17 compiler):

    pip install pybind11
    python setup.py build_ext --inplace

On Windows without the Windows SDK, use `python cpp/build_zig.py` instead.
"""
from pybind11.setup_helpers import Pybind11Extension
from setuptools import setup

setup(
    name="planner",
    ext_modules=[Pybind11Extension("planner._fastgrid", ["cpp/fastgrid.cpp"], cxx_std=17)],
)
