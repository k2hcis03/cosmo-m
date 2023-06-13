# kp:비례이득, ki:적분이득, kd:미분이득
class PID_Control:
 
    def __init__(self, _dt, _min, _max, _kp, _ki, _kd):
        self.update(_dt, _min, _max, _kp, _ki, _kd)
 
        self.pre_error = 0
        self.integral = 0
 
    def update(self, _dt, _min, _max, _kp, _ki, _kd):
        self.dt = _dt
        self.min = _min
        self.max = _max
        self.kp = _kp
        self.ki = _ki
        self.kd = _kd

        self.pre_error = 0
        self.integral = 0
    def calc(self, sv, pv):
        error = sv - pv
        # 비례
        kp = self.kp * error
 
        # 적분
        self.integral += error * self.dt
        ki = self.ki * self.integral
 
        # 미분
        kd = (error - self.pre_error) / self.dt
        kd = self.kd * kd
 
        # 합산
        result = kp + ki + kd
 
        if result > self.max:
            result = self.max
        elif result < self.min:
            result = self.min
 
        self.pre_error = error
 
        #description
        desc = f'Kp :\t{kp:.3f}\nKi :\t{ki:.3f}\nKd :\t{kd:.3f}\nPv :\t{pv:.3f}\nSv :\t{sv:.3f}'
 
        return result, desc