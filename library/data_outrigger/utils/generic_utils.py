from typing import Any


def istype(obj: Any, field: str, type) -> bool:
    return isinstance(getattr(obj, field, None), type)
