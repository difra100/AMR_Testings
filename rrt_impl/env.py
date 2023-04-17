class Env:
    def __init__(self, x_bounds, y_bounds):
        self.x_range = x_bounds
        self.y_range = y_bounds
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    def obs_boundary(self):
        obs_boundary = [
            [0, 0, 0.25, self.y_range[-1]],
            [0, self.y_range[-1], self.x_range[-1], 0.25],
            [0.25, 0, self.x_range[-1], 0.25],
            [self.x_range[-1], 0.25, 0.25, self.y_range[-1]]
        ]
        return obs_boundary

    def obs_rectangle(self):
        obs_rectangle = [
            [self.x_range[-1]//3, 12, 8, 2],
            [self.x_range[-1]//3, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]

        return obs_cir
