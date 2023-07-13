from pathlib import Path
from urllib.request import urlretrieve
from zipfile import ZipFile
from setuptools import setup, find_namespace_packages

from pybind11.setup_helpers import Pybind11Extension, build_ext

VERSION = "0.0.1"
DROW_DATA = "https://github.com/VisualComputingInstitute/DROW/releases/download/v2/DROWv2-data.zip"
DROW_WEIGHTS = "https://github.com/VisualComputingInstitute/DROW/releases/download/v2/final-WNet3xLF2p-T5-odom.rot-trainval-50ep.pth.tar"

dependencies = [
    "numpy~=1.24",
    "torch~=2.0",
    "tqdm~=4.64",
    "scipy~=1.11",
    "opencv-python~=4.8",
    "scikit-learn~=1.2",
    "matplotlib~=3.7"
]

ext_modules = [
    Pybind11Extension("cpp_binding",
        ["cpp_core/main.cpp"],
        define_macros = [("VERSION_INFO", VERSION)],
    ),
]



include_dir = Path("data_outrigger/include")
if not include_dir.exists():
    include_dir.mkdir(parents=True)
    file, _ = urlretrieve(DROW_DATA)
    with ZipFile(file, "r") as archive:
        archive.extractall(include_dir)
    urlretrieve(DROW_WEIGHTS, include_dir / Path("weights.pth.tar"))

setup(
    name="data_outrigger",
    version=VERSION,
    description="Library for all simple and commonly used ROS utilities I use.",
    author="Aleksandr Sergeev",
    author_email="alexander.sergeev@onmail.com",
    url='http://github.com/pseusys/follow_the_drow',
    packages=find_namespace_packages(),
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    install_requires=dependencies,
    include_package_data=True,
    python_requires=">=3.9",
)
