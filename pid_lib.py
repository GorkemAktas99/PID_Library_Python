class PID:

    def __init__(self, kp, ki, kd, stepsize):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.stepsize = stepsize

    def Derivative(self, ek, ek_1):
        self.ek = ek
        self.ek_1 = ek_1

        return (self.ek - self.ek_1) / self.stepsize

    def Integral(self, ek, ek_1):
        self.ek = ek
        self.ek_1 = ek_1

        return (self.ek + self.ek_1) * (self.stepsize / 2)

    def PID_Function(self, r, y):
        self.ek_1 = 0
        integralSum = 0
        self.r = r
        self.y = y
        self.ek = r-y
        integralSum = integralSum + self.Integral(self.ek, self.ek_1)
        self.outputPID = self.kp*self.ek+self.ki*integralSum+self.kd*self.Derivative(self.ek, self.ek_1)
        self.ek_1 = self.ek
        return self.outputPID
