import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import random

opposite_bounce = True
xlimit = 50
ylimit = 50

class Robot:
    def __init__(self):
        global xlmit, ylimit
        self.x = xlimit/2
        self.y = ylimit/2
        self.vx = random.randint(1, 5)
        self.vy = random.randint(1, 5)

    def move(self):
        if self.x >= xlimit or self.x <= 0:
            if opposite_bounce:
                self.vx *= -1
            else:
                self.vx = -1* random.random()
                self.vy = random.random()


        if self.y >= ylimit or self.y <= 0:
            if opposite_bounce:
                self.vy *= -1
            else:
                self.vy = -1 * random.random()
                self.vx = random.random()

        
        self.x += self.vx
        self.y += self.vy

    def show(self, d):
        d.set_data(self.x, self.y)

def animate(frame_number):

    robot.move()
    robot.show(d)
    

if __name__ == '__main__':
    robot = Robot()
    fig = plt.gcf()
    ax = plt.axes(xlim=(0,xlimit), ylim=(0,ylimit))
    d, = plt.plot(robot.x, robot.y, 'go')
    anim = animation.FuncAnimation(fig, animate, frames=200, interval=10, repeat=True)

    anim.save("robot-particle-opposite-bounce.gif", fps=24)