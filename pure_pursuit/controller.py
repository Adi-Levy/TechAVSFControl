import math

class PurePursuitController:
    """
        this a class to define a controller object of a pure pursuite controller
        
        the controller gets
            the current state of the car(x,y coordinates, velocity and current angle),
            the currnet path that the car should follow
        and then calculates the steering angle requiered to course correct according to the car's velocity
        in order to update the data in the controller it's update methods need to be called.
    """
    
    path = None
    coordinates = []
    orientation = 0
    velocity = 0
    _car_length = 0 
    _max_steering_angle = 0  # based on vehicle data and physical limits
    _kdd = 1                 # ld=kdd*v, ld=lookahead distance

    def __init__(self, car_length, kdd, msa): # initializes the controller and sets the initial values
        self._car_length = car_length
        self._kdd = kdd
        self._max_steering_angle = msa
            
    def update_state(self, x, y, v, orientation): # update the current state of the car in the controller for calculations
        self.coordinates = [x, y]
        self.velocity = v
        self.orientation = orientation

    def update_path(self, path): # update the path for the controller for calculations
        self.path = path

    def calculate_steering(self): # calc the needed steering angle to course correct to the next waypoint
        look_ahead_point = self._calculate_look_ahead_point()

        alpha = math.atan2(look_ahead_point[1] - self.coordinates[1], look_ahead_point[0] - self.coordinates[0]) - self.orientation # error angle
        delta = math.atan2(2*self.car_length*math.sin(alpha), ld)
        return max(delta, -self.max_steering_angle) if (delta < 0) else min(delta, self.max_steering_angle)

    def _calculate_look_ahead_point(self):
        ld = self.velocity * self.kdd
        # complete logic here
        
