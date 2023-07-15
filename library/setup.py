from pathlib import Path
from urllib.request import urlretrieve
from zipfile import ZipFile
from setuptools import setup

from pybind11.setup_helpers import Pybind11Extension, build_ext

VERSION = "0.0.1"
DROW_DATA = "https://github.com/VisualComputingInstitute/DROW/releases/download/v2/DROWv2-data.zip"
DROW_WEIGHTS = "https://github.com/VisualComputingInstitute/DROW/releases/download/v2/final-WNet3xLF2p-T5-odom.rot-trainval-50ep.pth.tar"

ext_modules = [
    Pybind11Extension(
        "data_outrigger.cpp_binding",
        [str(file) for file in Path("cpp_core").glob("*.cpp")],
        define_macros=[("VERSION_INFO", VERSION)],
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
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)
