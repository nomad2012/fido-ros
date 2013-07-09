#The recipe gives simple implementation of a Discrete Proportional-Integral-Derivative (PID) controller. PID controller gives output value for error between desired reference input and measurement feedback to minimize error value.
#More information: http://en.wikipedia.org/wiki/PID_controller
#
#cnr437@gmail.com
#
# modified by nomadicnerd@gmail.com: fixed spelling per PEP-8; added output_min and output_max parameters 
#
#######	Example	#########
#
#p=PID(3.0,0.4,1.2)
#p.set_setpoint(5.0)
#while True:
#     pid = p.update(measurement_value)
#
#


class PID(object):
    """
    Discrete PID control
    """

    def __init__(self, kP=2.0, kI=0.0, kD=1.0, output_min=-100, output_max=100, derivator=0, integrator=0, integrator_max=500, integrator_min=-500):

        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.output_min = output_min
        self.output_max = output_max
        self.derivator = derivator
        self.integrator = integrator
        self.integrator_max = integrator_max
        self.integrator_min = integrator_min
        self.setpoint = 0.0
        self.error = 0.0

    def update(self, current_value):
        """
        Calculate PID output value for given reference input and feedback
        """
        self.error = self.setpoint - current_value
        self.P_value = self.kP * self.error
        self.D_value = self.kD * ( self.error - self.derivator)
        self.derivator = self.error
        self.integrator = min(max(self.integrator + self.error, integrator_min), integrator_max)
        self.I_value = self.integrator * self.kI
        output = min(max(self.P_value + self.I_value + self.D_value, output_min), output_max)
        return output

    def set_setpoint(self, setpoint):
        """
        Initialize the setpoint of the PID
        """
        self.setpoint = setpoint
        self.integrator = 0
        self.derivator = 0
        
    def set_integrator(self, integrator):
        self.integrator = integrator
        
    def set_derivator(self, derivator):
        self.derivator = derivator

    def set_kP(self, kP):
        self.kP = kP

    def set_kI(self, kI):
        self.kI = kI

    def set_kD(self, kD):
        self.kD = kD

    def get_setpoint(self):
        return self.setpoint

    def get_error(self):
        return self.error

    def get_integrator(self):
        return self.integrator

    def get_derivator(self):
        return self.derivator
