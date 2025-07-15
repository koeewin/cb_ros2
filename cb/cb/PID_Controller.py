# pid_controller.py

class PIDController:
    def __init__(self, p, iTs, lower_limit=float('-inf'), upper_limit=float('inf'), aw=0.0):
        self.p = p
        self.iTs = iTs
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit
        self.aw = aw

        self.u = 0.0
        self.xP = 0.0
        self.xI = 0.0
        self.y = 0.0
        self.ySat = 0.0

    def put(self, u):
        self.u = u

    def run(self):
        self.xP = self.p * self.u
        self.xI += self.iTs * self.u + self.aw * (self.ySat - self.y)
        self.y = self.xP + self.xI
        self.ySat = self._limit(self.y)

    def get(self):
        return self.ySat

    def reset(self):
        self.xP = 0.0
        self.xI = 0.0
        self.y = 0.0
        self.ySat = 0.0

    def _limit(self, x):
        return max(min(x, self.upper_limit), self.lower_limit)
