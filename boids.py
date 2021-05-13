import numpy as np
from scipy.spatial.distance import squareform, pdist, cdist
import math

width, height = 640, 480


class Boids:
    # class to represent the Boids simulation

    def __init__(self, N, x= 300, y= 300, r = 10):
        """

        :param N: number of birds
        """
        self.pos = [width / 2.0, height / 2.0] + 10 * np.random.rand(2 * N).reshape(N, 2) # random normal distribution

        angles = 2 * math.pi * np.random.rand(N)
        self.velocity = np.array(list(zip(np.sin(angles), np.cos(angles))))
        self.obstacle_pos = ((x, y, r), (20,50,10))

        self.N = N

        self.min_dist = 25.0 # minimum distance of approach
        self.max_rule_velocity = 0.03
        self.max_velocity = 2.0

    def tick(self, frame_num, body, head):
        self.dist_matrix = squareform(pdist(self.pos))     # distance pair formation

        self.velocity += self.apply_rules()
        # self.velocity += self.avoid_obstacle()
        self.limit(self.velocity, self.max_velocity)
        self.pos += self.velocity
        self.apply_bc()

        body.set_data(self.pos.reshape(2 * self.N)[::2],
                      self.pos.reshape(2 * self.N)[1::2])
        velocity = self.pos + 10 * self.velocity/self.max_velocity
        head.set_data(velocity.reshape(2 * self.N)[::2],
                      velocity.reshape(2 * self.N)[1::2])

    def limit_velocity(self, velocity, max_value):
        magnitude = np.linalg.norm(velocity)
        if magnitude > max_value:
            velocity[0], velocity[1] = velocity[0] * max_value / magnitude, velocity[1] * max_value / magnitude

    def limit(self, X, max_value):
        for velocity in X:
            self.limit_velocity(velocity, max_value)

    def apply_bc(self):
        # applies boundary conditions
        delta_r = 2.0  # gives some time before moving boid to other edge of screen, smoothens animation look
        for coordinate in self.pos:
            if coordinate[0] > width + delta_r:
                coordinate[0] = - delta_r
            elif coordinate[0] < -delta_r:
                coordinate[0] = width + delta_r
            if coordinate[1] > height + delta_r:
                coordinate[1] = -delta_r
            elif coordinate[1] < -delta_r:
                coordinate[1] = height + delta_r

    def apply_rules(self):
        D = self.dist_matrix < 25.0  # Rule 1: max separation         Forms boolean numpy array
        velocity = self.pos * D.sum(axis=1) .reshape(self.N, 1) - D.dot(self.pos)
        self.limit(velocity, self.max_rule_velocity)

        D = self.dist_matrix < 50  # alignment distance threshold

        velocity_2 = D.dot(self.velocity)  # Rule 2: alignment of birds   D.dot is the dot product
        self.limit(velocity_2, self.max_rule_velocity)
        velocity += velocity_2

        velocity_3 = D.dot(self.pos) - self.pos  # Rule 3: Cohesion of flock
        self.limit(velocity_3, self.max_rule_velocity)
        velocity += velocity_3

        return velocity
    """
    def avoid_obstacle(self):
        for obs in self.obstacle_pos:
            D = np.linalg.norm(self.pos, np.array(obs[0], obs[1]).reshape(self.N, 2)) < obs[2]
            velocity = 0.1 * (self.pos * D.sum(axis=1) .reshape(self.N, 1) - D.dot(self.pos))
            # self.limit(velocity, self.max_rule_velocity)

        return velocity
    
    def button_press(self, event):
        if event.button is 1:   # left-click: event.button = 1
            self.pos = np.concatenate((self.pos, np.array([[event.xdata, event.ydata]])), axis=0)  # axis = 0 default
            # concatenate combines the arrays
            angles = 2 * math.pi * np.random.rand(1)
            v = np.array(list(zip(np.sin(angles), np.cos(angles))))
            self.velocity = np.concatenate((self.velocity, v), axis=0) # axis already default set to 0
            self.N += 1
        elif event.button is 3:
            self.velocity += 0.1 * (self.pos - np.array([[event.xdata, event.ydata]]))
    """

def tick(frame_num, body, head, boids):
    # print(frame_num)
    # FuncAnimation update function
    boids.tick(frame_num, body, head)
    return body, head



