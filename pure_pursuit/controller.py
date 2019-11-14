import math
import numpy as np

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
    _poly_deg = 3

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
        delta = math.atan2(2*self._car_length*math.sin(alpha), self._point_distance(look_ahead_point))
        return max(delta, -self._max_steering_angle) if (delta < 0) else min(delta, self._max_steering_angle)

    def _calculate_look_ahead_point(self):
        ld = self.velocity * self._kdd
        
        near_point_index = self._find_near_point_index()

        swapped_axis_path = np.swapaxes(self.path[max(near_point_index-4, 0):min(near_point_index+4, len(self.path)-1)], 0 , 1)
        p = np.poly1d(np.polyfit(swapped_axis_path[0], swapped_axis_path[1], self._poly_deg))

        point_index = 0
        while (self._point_distance(self.path[near_point_index+point_index]) < ld):
            point_index+=1
        
        x_sector = np.linspace(self.path[(near_point_index+point_index-1)[0]], self.path[near_point_index+point_index[0]], 5)
        y_sector = np.polyval(p, x_sector)
        
        point_index = 0
        while (self._point_distance([x_sector[point_index], y_sector[point_index]]) < ld):
            point_index+=1

        return [x_sector[point_index], y_sector[point_index]]

    def _find_near_point_index(self):
        near_point = [self.path[0]]
        for way_point in self.path:
            near_point_distance = self._point_distance(near_point)
            way_point_distance = self._point_distance(way_point)
            if (near_point_distance > way_point_distance):
                near_point = way_point
        
        return self.path.index(near_point)

    def _point_distance(self, point):
        return math.sqrt((self.coordinates[0] - point[0])**2 + (self.coordinates[1] - point[1])**2)

        
