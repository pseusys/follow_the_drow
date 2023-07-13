from typing import Any


def istype(obj: Any, field: str, type) -> bool:
    return isinstance(getattr(obj, field, None), type)


class Logging:
    def __init__(self, verbose: bool = True):
        self._verbose = verbose

    def _print(self, message: str):
        if self._verbose:
            print(message)

