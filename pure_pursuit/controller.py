import math

class PurePersuiteController:
    """
        this a class to define a controller object of a pure pursuite controller
        
        the controller gets
            the current state of the car(x,y coordinates, velocity and current angle),
            the next waypoint's coordinates (in the lookahead list)
        and then calculates the steering angle requiered to course correct according to the car's velocity
        in order to update the data in the controller it's update methods need to be called.
    """
    
    car_length = 0 
    look_ahead_point = [] # the point [x,y] on the path a lookahead distance away from the car
    coordinates = []
    orientation = 0
    velocity = 0
    max_steering_angle = 0 # based on vehicle data and physical limits
    kdd = 1 # ld=kdd*v, ld=lookahead distance

    def __init__(self, look_ahead_point, car_length, x0, y0, theta0, kdd, v, msa): # initializes the controller and sets the initial values
        self.look_ahead_point = look_ahead_point
        self.car_length = car_length
        self.coordinates = [x0, y0]
        self.orientation = theta0
        self.kdd = kdd
        self.velocity = v
        self.max_steering_angle = msa
            
    def update_state (self, x, y, v, orientation): # update the current state of the car in the controller for calculations
        self.coordinates = [x, y]
        self.velocity = v
        self.orientation = orientation

    def update_waypoints (self, look_ahead_point): # update the waypoints of the path for the controller for calculations
        self.look_ahead_point = look_ahead_point

    def suggest_steering(self): # calc the needed steering angle to course correct to the next waypoint
        ld = self.velocity * self.kdd 
        alpha = math.atan2(self.look_ahead_point[1] - self.coordinates[1], self.look_ahead_point[0] - self.coordinates[0]) - self.orientation # error angle
        delta = math.atan2(2*self.car_length*math.sin(alpha), ld)
        return max(delta, -self.max_steering_angle) if (delta < 0) else min(delta, self.max_steering_angle)
        
