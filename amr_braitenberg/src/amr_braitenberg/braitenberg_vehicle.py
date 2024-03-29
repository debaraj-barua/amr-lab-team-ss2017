#!/usr/bin/env python


class BraitenbergVehicle:

    TYPE_A = 0  # direct connections
    TYPE_B = 1  # cross connections
    TYPE_C = 2  # direct and cross connections

    def __init__(self, *args):
        """
        init with default params (type A, factor 1.0)
        """
        self.set_params()
        pass


    def set_params(self, vehicle_type=TYPE_A, factor_1=1.0, factor_2=1.0):
        self._vehicle_type = vehicle_type
        self._f_1, self._f_2 = factor_1, factor_2


    def compute_wheel_speeds(self, left_in, right_in):
        """
        ==================== YOUR CODE HERE ====================
        Instructions: based on the input from the left and
                      right sonars compute the speeds of the
                      wheels. Use the parameters stored in the
                      private fields self._vehicle_type, self._f_1, and
                      self._f_2 (if applicable).

        Hint: a good idea would be to pass here the normalized sonar
        readings scaled by maximum range, i.e. proximity to an obstacle
        (in interval [0..1])
        ========================================================
        """
        max_float32 = (2^31) * 1.

        if(self._vehicle_type == 0):
            # Set speed so that it goes away from obstacles
            speed_left = (1./left_in) * self._f_1 if left_in > 0 else max_float32
            speed_right = (1./right_in) * self._f_2 if right_in > 0 else max_float32
        elif(self._vehicle_type == 1):
            # Set speed to hit obstacle
            speed_left = left_in * self._f_1
            speed_right = right_in * self._f_2
        else:
            # Sensor influences both wheels and the vehicle will go straight
            speed_left = (left_in + right_in) * self._f_1
            speed_right = (right_in + left_in) * self._f_2

        return (speed_left, speed_right)
