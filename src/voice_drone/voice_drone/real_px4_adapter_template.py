class RealPx4Adapter:
    def __init__(self):
        raise RuntimeError('left blank on purpose in the safe build')

    def arm_takeoff(self, alt_m: float) -> None:
        raise RuntimeError('left blank on purpose in the safe build')

    def land(self) -> None:
        raise RuntimeError('left blank on purpose in the safe build')

    def hold(self) -> None:
        raise RuntimeError('left blank on purpose in the safe build')

    def move_body(self, x_m: float, y_m: float, z_m: float) -> None:
        raise RuntimeError('left blank on purpose in the safe build')

    def rotate(self, yaw_deg: float) -> None:
        raise RuntimeError('left blank on purpose in the safe build')
