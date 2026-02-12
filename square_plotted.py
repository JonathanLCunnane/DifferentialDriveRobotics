from baselib.base import BP, RIGHT_M, LEFT_M, move, rotate, init_BP, DEG_TO_RAD
from time import sleep
from math import cos, sin
from random import gauss
from collections import namedtuple

STOP_TIME = 1
N_PARTICLES = 100
LINE_OFFSET = 200
LINE_SCALE = 10
BOX_SIZE = 40
MAX_Y = BOX_SIZE * LINE_SCALE

Particle = namedtuple("Particle", ["x", "y", "theta"])

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

def draw_particles(particles: list[tuple[float, float, float]]):
    particles = [
        tuple((*adj_coord(x, y), theta))
        for (x, y, theta) in particles
    ]
    print(f"drawParticles:{particles}")

def draw_square():
    draw_line((0, 0, 40, 0))
    draw_line((0, 0, 0, 40))
    draw_line((0, 40, 40, 40))
    draw_line((40, 0, 40, 40))

class ParticleSet:
    def __init__(self):
        self.particles = [Particle(0, 0, 0) for _ in range(N_PARTICLES)]
        self.weights = [1/N_PARTICLES for _ in range(N_PARTICLES)]
        self.e_sig = 1/16
        self.f_sig = (2.5 * DEG_TO_RAD) / 16
        self.g_sig = 3 * DEG_TO_RAD

    def plot(self):
        draw_particles(self.particles)

    def forward(self, cm: float):
        new_particles = []
        
        for x, y, theta in self.particles:
            e_gauss = gauss(0, self.e_sig)
            new_particles.append(Particle(
                x + (cm + e_gauss) * cos(theta),
                y + (cm + e_gauss) * sin(theta),
                theta + gauss(0, self.f_sig)
            ))
        self.particles = new_particles

    def rotate(self, deg: float):
        new_particles = []
        rad = deg * DEG_TO_RAD
        
        for x, y, theta in self.particles:
            new_particles.append(Particle(
                x,
                y, 
                theta + gauss(0, self.g_sig) + rad
            ))
        self.particles = new_particles

try:
    init_BP()
    draw_square()
    particle_set = ParticleSet()
    particle_set.plot()

    for _ in range(4):
        
        for _ in range(4):
            move(10, dist_const=(40/34))
            particle_set.forward(10)
            particle_set.plot()
            sleep(STOP_TIME)
            
        rotate(90, rotate_const=(90/74))
        particle_set.rotate(90)
        particle_set.plot()
        
        sleep(STOP_TIME)
except KeyboardInterrupt:
    BP.reset_all()
except Exception as e:
    print(e)
finally:
    BP.reset_all()



