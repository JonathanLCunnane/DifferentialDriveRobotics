from baselib.base import BP, RIGHT_M, LEFT_M, move, rotate, init_BP, DEG_TO_RAD, EPSILON, waypoint
from time import sleep
from math import cos, sin, exp, atan2
from random import gauss, random
from collections import namedtuple
import traceback

STOP_TIME = 1
N_PARTICLES = 100
LINE_OFFSET = 50
LINE_SCALE = 3
BOX_SIZE = 210
MAX_Y = BOX_SIZE * LINE_SCALE
SONAR_S = BP.PORT_1

START_X = 84
START_Y = 30
START_T = 0

SONAR_OFFSET = 0

STEP_SIZE = 20

WP_EPSILON = 2
EPSILON = 0.1

labmap = [
    (0, 0, 0, 168),     # Wall O-A
    (0, 168, 84, 168),  # Wall A-B
    (84, 126, 84, 210), # Wall C-D
    (84, 210, 168, 210),# Wall D-E
    (168, 210, 168, 84),# Wall E-F
    (168, 84, 210, 84), # Wall F-G
    (210, 84, 210, 0),  # Wall G-H
    (210, 0, 0, 0)      # Wall H-O
]

WAYPOINTS = [
    (84, 30),
    (180, 30),
    (180, 54),
    (138, 54),
    (138, 168),
    (114, 168),
    (114, 84),
    (84, 84),
    (84, 30),
]

START = WAYPOINTS[0]

class Particle:
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight
        self.iter = iter([x, y, theta, weight])

    def __iter__(self):
        self.iter = iter([self.x, self.y, self.theta, self.weight])
        return self

    def __next__(self):
        return next(self.iter)

def adj_coord(x: float, y: float):
    new_x = LINE_SCALE * x + LINE_OFFSET
    new_y = (MAX_Y - (LINE_SCALE * y)) + LINE_OFFSET
    return new_x, new_y

def draw_line(line: tuple[float, float, float, float]):
    new_0 = adj_coord(line[0], line[1])
    new_1 = adj_coord(line[2], line[3])
    line = new_0 + new_1
    print(line)
    print(f"drawLine:{line}")

def draw_lines(lines: list[tuple[float, float, float, float]]):
    for line in lines:
        draw_line(line)

def draw_particles(particles: list[Particle]):
    particles = [
        tuple((*adj_coord(p.x, p.y), p.theta))
        for p in particles
    ]
    print(f"drawParticles:{particles}")

class ParticleSet:
    def __init__(self, start_x, start_y, start_t):
        self.particles = [Particle(start_x, start_y, start_t, 1/N_PARTICLES) for _ in range(N_PARTICLES)]
        self.e_sig = 1/16
        self.f_sig = (2.5 * DEG_TO_RAD) / 16
        self.g_sig = 3 * DEG_TO_RAD
        self.z_sig = 3
        self.k = 0.05

    def plot(self):
        draw_particles(self.particles)

    def forward(self, cm: float):
        new_particles = []
        
        for x, y, theta, weight in self.particles:
            e_gauss = gauss(0, self.e_sig)
            new_particles.append(Particle(
                x + (cm + e_gauss) * cos(theta),
                y + (cm + e_gauss) * sin(theta),
                theta + gauss(0, self.f_sig),
                weight
            ))
        self.particles = new_particles

    def rotate(self, deg: float):
        new_particles = []
        rad = deg * DEG_TO_RAD
        
        for x, y, theta, weight in self.particles:
            new_particles.append(Particle(
                x,
                y, 
                theta + gauss(0, self.g_sig) + rad,
                weight
            ))
        self.particles = new_particles
        
    def update_weights(self, sonar_reading):
        total_weight = 0
        for p in self.particles:
            likelihood = self.calculate_likelihood(p.x, p.y, p.theta, sonar_reading)
            p.weight *= likelihood
            total_weight += p.weight
        
        # Normalisation
        if total_weight == 0: total_weight = 1e-10 # Prevent div by zero
        for p in self.particles:
            p.weight /= total_weight
        
    def calculate_likelihood(self, x, y, theta, z):
        z_corrected = z + SONAR_OFFSET

        min_m = 1000 
        
        for (Ax, Ay, Bx, By) in labmap:
            num = (By - Ay) * (Ax - x) - (Bx - Ax) * (Ay - y)
            denom = (By - Ay) * cos(theta) - (Bx - Ax) * sin(theta)

            if denom == 0: continue 

            m = num / denom

            if m <= 0: continue 

            hit_x = x + m * cos(theta)
            hit_y = y + m * sin(theta)

            if (min(Ax, Bx) - EPSILON <= hit_x <= max(Ax, Bx) + EPSILON) and (min(Ay, By) - EPSILON <= hit_y <= max(Ay, By) + EPSILON):
                if m < min_m:
                    min_m = m

        m = min_m
        likelihood = exp(-((z_corrected - m)**2) / (2 * (self.z_sig**2))) + self.k
        return likelihood
    
    def resample(self):
        new_particles = []
        cumulative_weights = []
        cum_sum = 0
        
        # Build cumulative distribution
        for p in self.particles:
            cum_sum += p.weight
            cumulative_weights.append(cum_sum)
            
        # Select particles
        for _ in range(N_PARTICLES):
            r = random() # 0.0 to 1.0
            # Find index where r fits
            for i, cw in enumerate(cumulative_weights):
                if r <= cw:
                    orig = self.particles[i]
                    new_particles.append(Particle(orig.x, orig.y, orig.theta, 1.0/N_PARTICLES))
                    break
        
        self.particles = new_particles
        
    def get_estimate(self):
        # get a mean (estimate where robot is based on particles)
        x_sum = 0
        y_sum = 0
        cos_sum = 0
        sin_sum = 0
        
        for p in self.particles:
            x_sum += p.x * p.weight
            y_sum += p.y * p.weight
            cos_sum += cos(p.theta) * p.weight
            sin_sum += sin(p.theta) * p.weight
            
        return x_sum, y_sum, atan2(sin_sum, cos_sum)

try:
    init_BP()
    BP.set_sensor_type(SONAR_S, BP.SENSOR_TYPE.NXT_ULTRASONIC)
    draw_lines(labmap)
    start_x, start_y = START
    particle_set = ParticleSet(start_x, start_y, 0)
    particle_set.plot()
       
    for wp in WAYPOINTS:

        while (moved_dist := None) is None or moved_dist > WP_EPSILON:
            curr_x, curr_y, curr_theta = particle_set.get_estimate()
            print("Estimated x, y, theta: ", curr_x, curr_y, curr_theta)
            
            next_x, next_y, next_theta = waypoint((curr_x, curr_y, curr_theta), wp, STEP_SIZE, False)
            
            moved_dist = ((next_x - curr_x) ** 2 + (next_y - curr_y) ** 2) ** 0.5


            particle_set.rotate(next_theta - curr_theta)
            particle_set.forward(moved_dist)
            particle_set.plot()

            z = BP.get_sensor(SONAR_S)

            particle_set.update_weights(z)
            particle_set.resample()

            sleep(STOP_TIME) 
    
except KeyboardInterrupt:
    BP.reset_all()
except Exception as e:
    print(traceback.format_exc())
finally:
    BP.reset_all()



