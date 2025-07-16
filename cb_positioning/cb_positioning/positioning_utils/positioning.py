"""
@file       positioning.py
@details    This scripts implements the parent class for the `positioning_vision_human.PositioningVisionHuman` and 
            the `positioning_vision_apriltag.PositioningVisionAprilTag` classes.
"""

class Positioning:
    """
    A class for representing and managing positioning data.

    This class tracks distance, angle, Cartesian coordinates (x, y), 
    and a command state. It is intended to be extended for specific 
    positioning logic via the @ref run method.
    """

    def __init__(self):
        """
        Initializes the Positioning object with default values:
        distance, angle, x, y as 0.0, and command as 0.
        """

        ##  @name Distance and angle relative to the robot
        #   @{

        ## Distance from the robot
        self._distance = 0.0  

        ## Angle in radians between the robot's orientation and the line 
        #  connecting the human and the robot 
        self._angle = 0.0    

        ## @}

        ##  @name Cartesian coordinates in the robot's frame
        #   @{

        ## X-coordinate 
        self._x = 0.0      

        ## Y-coordinate   
        self._y = 0.0   

        ## @}

        ## Command/state identifier
        self._command = 0     

    def get_distance(self):
        """
        Gets the current distance value.

        @return Distance as a float.
        """
        return self._distance

    def get_angle(self):
        """
        Gets the current angle value.

        @return Angle as a float.
        """
        return self._angle

    def get_x(self):
        """
        Gets the current X-coordinate.

        @return X-coordinate as a float.
        """
        return self._x

    def get_y(self):
        """
        Gets the current Y-coordinate.

        @return Y-coordinate as a float.
        """
        return self._y

    def get_command(self):
        """
        Gets the current command value.

        @return Command as an integer.
        """
        return self._command

    def run(self):
        """
        Placeholder for the main execution logic.

        Override this method to implement specific behavior.
        """
        pass