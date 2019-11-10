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
    
    L = 0 #Car Length
    lookAhead = [] #mx2
    coordinates = []
    theta = 0
    velocity = 0
    maxSteeringAngle = 0 # based on vehicle data and physical limits
    Kdd = 1 #ld=Kdd*v, ld=lookahead distance

    def __init__(self, WayPoints, L, x0, y0, Theta0, Kdd, V, msa): # initializes the controller and sets the initial values
        self.lookAhead = WayPoints
        self.L = L
        self.coordinates = [x0, y0]
        self.theta = Theta0
        self.Kdd = Kdd
        self.velocity = V
        self.maxSteeringAngle = msa
            
    def updateState (self,x,y,v,theta): # update the current state of the car in the controller for calculations
        self.coordinates = [x, y]
        self.velocity = v
        self.theta = theta

    def updateWaypoints (self, waypoints): # update the waypoints of the path for the controller for calculations
        self.lookAhead = waypoints

    def SuggestSteering(self): # calc the needed steering angle to course correct to the next waypoint
        ld = self.velocity * self.Kdd 
        alpha = math.atan2(self.lookAhead[1] - self.coordinates[1], self.lookAhead[0] - self.coordinates[0]) - self.theta #error angle
        delta = math.atan2(2*self.L*math.sin(alpha),ld)
        return max(delta, -self.maxSteeringAngle) if (delta < 0) else min(delta, self.maxSteeringAngle)
        
