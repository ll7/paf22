import matplotlib.pyplot as plt
from typing import Tuple, List


class TestVisualizer:
    def __init__(self, start: Tuple[float, float], end: list,
                 x: List[float], y: List[float],
                 x_ref: List[float], y_ref: List[float]):
        self.start = start
        self.end = end
        self.x = x
        self.y = y
        self.x_ref = x_ref
        self.y_ref = y_ref

    def visualize(self):
        plt.xlabel("x coordinates")
        plt.ylabel("y coordinates")
        plt.title("2D Trajectory")
        plt.scatter(self.x, self.y)
        plt.scatter(self.x_ref, self.y_ref, c="green")
        plt.scatter(self.start[0], self.start[1], c="red", marker="o")
        plt.scatter(self.end[0], self.end[1], c="orange", marker="o")
        # plt.xlim(980, 1030)
        # plt.ylim(-5550, -5600)
        plt.show()
