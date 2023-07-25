from tqdm.auto import trange

from ..datasets import DROW_Dataset
from ..utils.drow_utils import laser_minimum, laser_increment
from ..cpp_binding import DetectorFactory, DetectorType

from .detector import Detector


class AlgorithmicDetector(Detector):
    N_TIME = 5

    def __init__(self, type: DetectorType, verbose: bool = True, *args, **kwargs):
        Detector.__init__(self, verbose)
        self._detector = DetectorFactory(type, laser_minimum, laser_increment)

    def forward_one(self, xb):
        return self._detector.forward_one(xb)

    def forward_all(self, va: DROW_Dataset):
        people = list()
        for iseq in trange(len(va.det_id), desc="Sequences", disable=not self._verbose):
            for idet in trange(len(va.det_id[iseq]), desc="Scans", disable=not self._verbose, leave=False):
                iscan = va.idet2iscan[iseq][idet]
                scans, odoms = va.get_scan(iseq, iscan, self.N_TIME)
                people += [self.forward_one(scans[-1])]
        return people
