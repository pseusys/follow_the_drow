from enum import Enum, unique
from math import sqrt


@unique
class TrackerPolicy(Enum):
    First = "first"
    Tracked = "tracked"
    Closest = "closest"

    def __str__(self):
        return self.value
    
    @staticmethod
    def distance(a, b) -> float:
        return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def track(self, points, previous = (0, 0)):
        if len(points) < 0:
            raise RuntimeError("Points list is empty!")
        if self is self.First:
            return points[0]
        elif self is self.Tracked:
            return min(points, key=lambda e: self.distance(e, previous))
        elif self is self.Closest:
            return min(points, key=lambda e: self.distance(e, (0, 0)))
        else:
            raise RuntimeError(f"Bad tracker policy value '{self}'!")
