import math
try:
    from .positioning import Positioning  # Relative import within the package
except ImportError:
    from positioning import Positioning  # Normal import when running directly

# Parameters for fusion process
UWB_PHI_CORRECTION = 0.0
UWB_D_CORRECTION = -0.055
VIS_PHI_CORRECTION = 0.0
VIS_D_CORRECTION = 0.0

ONE_OVER_SIGMA_PHI_UWB_SQUARED = 670
ONE_OVER_SIGMA_D_UWB_SQUARED = 7932
ONE_OVER_SIGMA_PHI_VISION_SQUARED = 62255
ONE_OVER_SIGMA_D_VISION_SQUARED = 3743


class PositioningFusion(Positioning):
    D_STOP = 0.65  # Class attribute

    def __init__(self, **options):
        """
        Initialize the Fusion Positioning.

        Parameters:
        - options (dict): Additional optional parameters.
        """
        super().__init__()

        # Correction values for UWB and Vision sensors
        self._uwb_phi_correction = options.get('uwb_phi_correction', UWB_PHI_CORRECTION)
        self._uwb_d_correction = options.get('uwb_d_correction', UWB_D_CORRECTION)
        self._vis_phi_correction = options.get('vis_phi_correction', VIS_PHI_CORRECTION)
        self._vis_d_correction = options.get('vis_d_correction', VIS_D_CORRECTION)

        # Sensor measurement weights (1 / sigma^2)
        self._s_phi_u = options.get('s_phi_u', ONE_OVER_SIGMA_PHI_UWB_SQUARED)
        self._s_d_u = options.get('s_d_u', ONE_OVER_SIGMA_D_UWB_SQUARED)
        self._s_phi_v = options.get('s_phi_v', ONE_OVER_SIGMA_PHI_VISION_SQUARED)
        self._s_d_v = options.get('s_d_v', ONE_OVER_SIGMA_D_VISION_SQUARED)

        # Pre-calculated sums for fusion
        self._sum_s_phi = self._s_phi_u + self._s_phi_v
        self._sum_s_d = self._s_d_u + self._s_d_v

        # Measurements from UWB and Vision sensors
        self._phi_u = 0.0
        self._d_u = 0.0
        self._phi_v = 0.0
        self._d_v = 0.0

    def run(self):
        self.fuse_polar()
        self.calculate_xy()

    def put_measurements(self, phi_u, d_u, phi_v, d_v):
        """Accepts current measurements from the two sensors and applies biases."""
        # UWB measurements with corrections
        self._phi_u = phi_u + self._uwb_phi_correction
        self._d_u = d_u + self._uwb_d_correction

        # Vision measurements with corrections
        self._phi_v = phi_v + self._vis_phi_correction
        self._d_v = d_v + self._vis_d_correction

    def fuse_polar(self):
        """Performs sensor fusion on polar coordinates from UWB and Vision sensors."""
        if self._d_v < 0.31: #0.1: change to 0.31 because of the OFFSET_X_VISION 
            # Vision sensor doesn't detect; rely on UWB
            self._distance = self._d_u
            self._angle = self._phi_u
        elif abs(self._phi_u - self._phi_v) > 0.5:
            # Significant angle difference; rely on UWB
            self._distance = self._d_u
            self._angle = self._phi_u
        elif self._d_v < self.D_STOP:
            # Operator is too close to the camera; stop the robot
            self._distance = self.D_STOP / 2
            self._angle = 0
        elif self._d_v > 30:
            # Operator is too far from the camera; slow down
            self._distance = self.D_STOP / 3
            self._angle = 0
        else:
            # Fuse data from both sensors
            self._angle = ((self._s_phi_u * self._phi_u) + (self._s_phi_v * self._phi_v)) / self._sum_s_phi
            self._distance = ((self._s_d_u * self._d_u) + (self._s_d_v * self._d_v)) / self._sum_s_d

    def calculate_xy(self):
        """Converts polar coordinates (distance, angle) into Cartesian coordinates (x, y)."""
        self._x = round(self._distance * math.cos(self._angle), 3)
        self._y = round(self._distance * math.sin(self._angle), 3)



