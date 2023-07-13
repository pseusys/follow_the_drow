from abc import abstractmethod

from ..datasets.dataset import Dataset
from ..utils.generic_utils import Logging


class Detector(Logging):
    def __init__(self, verbose: bool = True):
        Logging.__init__(self, verbose)

    @classmethod
    @abstractmethod
    def init(cls, verbose: bool = True) -> "Detector":
        raise NotImplementedError

    # TODO: single input and output interface
    @abstractmethod
    def forward_one(self, xb):
        raise NotImplementedError

     # TODO: single input and output interface
    @abstractmethod
    def forward_all(self, va: Dataset):
        raise NotImplementedError
