# -*- coding: utf-8 -*-
"""
Created on Fri Apr 21 16:18:53 2017

@author: David
"""

import pygame
from vec2d import Vec2d
import random
import math

RED = (255,0,0)
GREEN = (0,255,0)
BLUE = (0,0,255)
CYAN = (0,255,255)
MAGENTA = (255,0,255)
YELLOW = (255,255,0)
WHITE = (255,255,255)
BLACK = (0,0,0)

enemiesDead = 0

class Shape:
    def __init__(self, pos, vel, angle, angvel, color, mass, moment, points):
        self.pos = Vec2d(pos)
        self.vel = Vec2d(vel)
        self.force = Vec2d(0,0)
        self.color = color
        self.origpoints = []
        self.points = []
        for p in points:
            self.origpoints.append(Vec2d(p))
            self.points.append(Vec2d(p))
        self.create_origaxes()
        self.angle = angle
        self.angvel = float(angvel)
        self.torque = 0.0
        self.mass = mass
        self.massinv = 1.0/mass
        self.moment = moment
        self.momentinv = 1.0/moment
        self.visible = True
        self.sleeping = False
        self.update_points()
        self.update_axes()    
        
    def create_origaxes(self):
        self.origaxes = []
        self.axes = []
        """ (5) Create origaxes from origpoints to use for collisions """
        for i in range(len(self.origpoints)):
            if i == 0:
                self.origaxes.append(self.origpoints[0] - self.origpoints[len(self.origpoints) - 1])
                self.axes.append(self.origaxes[0])
            else:
                self.origaxes.append(self.origpoints[i] - self.origpoints[i - 1])  
                self.axes.append(self.origaxes[i])
        
       
    def update_points(self):
        """ (4) Replace self.origpoints[i] with a calculation of the current 
                location of the point based on pos and angle. """
        for i in range(len(self.origpoints)):
            self.points[i] = Vec2d(((math.cos(self.angle) * self.origpoints[i].x) + ((-1*(math.sin(self.angle))) * self.origpoints[i].y)), ((math.sin(self.angle) * self.origpoints[i].x) + ((math.cos(self.angle))) * self.origpoints[i].y)) + self.pos
            
    def update_axes(self):
        """ (6) Replace self.origaxes[i] with a calculation of the current 
                axis direction based on the current angle. """
        for i in range(len(self.origaxes)):
            self.axes[i] = Vec2d(((math.cos(self.angle) * self.origaxes[i].x) + ((-1*(math.sin(self.angle))) * self.origaxes[i].y)), ((math.sin(self.angle) * self.origaxes[i].x) + ((math.cos(self.angle))) * self.origaxes[i].y))
                    
    def add_impulse(self, imp, pos):
        """ (3) change vel and angvel based on added impulse """
        self.vel    += imp/self.mass
        self.angvel += (pos - self.pos).cross(imp) / self.moment
        
    def draw(self, screen):
        if self.visible:
            self.update_points()
            n = len(self.points)
            if n > 2:
                pygame.draw.polygon(screen, self.color, self.points, 0)       
            else:
                pygame.draw.line(screen, self.color, self.points[0], self.points[-1], 1)
            
class World:
    def __init__(self, width_pixels, height_pixels, bg_color = (255,255,255)):
        pygame.init()
        self.width = width_pixels
        self.height = height_pixels
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.bg_screen = pygame.Surface((self.width, self.height))
        self.bg_screen.fill(bg_color)
        self.shapes = []

    def add(self, shape):
        self.shapes.append(shape)
        
    def append(self, shape):
        self.shapes.append(shape)
        
    def prepend(self, shape):
        self.shapes.insert(0, shape)
        
    def remove(self, shape):
        self.shapes.remove(shape)
        
    def display(self):
        self.screen.blit(self.bg_screen, (0,0))
        for p in self.shapes:
            p.draw(self.screen)
        #pygame.display.flip()

    def display_noflip(self):
        self.screen.blit(self.bg_screen, (0,0))
        for p in self.shapes:
            p.draw(self.screen)

def update_pos(shapes, dt):
    """ (1) add angle update """
    for p in shapes:
        if not p.sleeping:
            p.pos   += p.vel*dt
            p.angle += p.angvel*dt
            p.sleeping = False

def update_vel(shapes, dt):
    """ (2) add angvel update """
    for p in shapes:
        if not p.sleeping:
            p.vel    += p.force*p.massinv*dt
            p.angvel += (p.torque/p.moment)*dt
        
def update_force(shapes, world):
    for p in shapes:
        p.force = Vec2d(0,0)
        p.torque = 0.0
    for p in shapes:
        gravity = Vec2d(0, 0.05)
        p.force += p.mass * gravity
        
class Wall:
    def __init__(self, normal,position, e):
        self.normal = normal
        self.pos = position
        self.e = e

def collide(shape1, shape2):
    collisionNormals = []
    flipList = []
    separationVector = []
    minimumOverlap = 100000
    moIndex = 0
    passCount = 0
    returnPoint = Vec2d(0, 0)

    for a in shape1.axes:
        n = a.perpendicular_normal()      
        collisionNormals.append(n)
    for a in shape2.axes:
        n = a.perpendicular_normal()      
        collisionNormals.append(n) 
        
    for n in collisionNormals:
        pDots = []
        qDots = []

        for i in range(0, len(shape1.points)):
            pDots.append(n.dot(shape1.points[i]))
            flipList.append(0);
        for i in range(0, len(shape2.points)):    
            qDots.append(n.dot(shape2.points[i]))
            flipList.append(0);
            
        pMin = min(pDots)
        pMax = max(pDots)
        qMin = min(qDots)
        qMax = max(qDots)

        if pMax <= qMin or pMin >= qMax:
            return (False, separationVector, returnPoint)
        else:
            if ((pMax - qMin) < (qMax - pMin)):
                if (pMax - qMin < minimumOverlap):
                    minimumOverlap = pMax - qMin
                    moIndex = passCount
                    returnPoint = shape1.points[pDots.index(pMax)]
                    flipList[moIndex] = 1
            if ((pMax - qMin) >= (qMax - pMin)):
                if (qMax - pMin < minimumOverlap):
                    minimumOverlap = qMax - pMin
                    returnPoint = shape2.points[qDots.index(qMax)]
                    moIndex = passCount
        
        passCount += 1
            
    #min_overlap = min(overlaps)
    axis = collisionNormals[moIndex]
    if (flipList[moIndex] == 1):
        separationVector = axis * minimumOverlap * -1
    else:
        separationVector = axis * minimumOverlap
    return (True, separationVector, returnPoint)
    
def remove_shapes(shapesToRemove, shapes, world):
    for i in range(len(shapesToRemove)):
        world.remove(shapesToRemove[i])
        shapes.remove(shapesToRemove[i])
        
def handle_collisions(shapes, world):
    e = 0.8
    walls = [Wall(Vec2d(0, -1), Vec2d(0, world.height), 0.5)]
    shapesToRemove = []
    sleep_vel = 0.005
    sleep_angvel = 0.001
    for i in range(len(shapes)):
        p = shapes[i]
        p.update_points()
        p.update_axes()
        pShapeRemoved = False
        for q in shapes:
            if p != q:
                #pointNull = Vec2d(0,0)
                if (p.points[0].x > 1700 or p.points[0].x < -200) and (p.color == RED or p.color == BLUE or p.color == GREEN) and pShapeRemoved == False:
                    pShapeRemoved = True
                    p.color = YELLOW
                    shapesToRemove.append(p)
                (colliding, sepVec, point) = collide(p, q)
                if colliding:
                    mu = 1 / (p.massinv + q.massinv)
                    #r = sepVec
                    n = sepVec.normalized()
                    v = p.vel - q.vel
                    pI = p.moment
                    qI = q.moment
                    #rcross = r.cross(n)
                    prxn = (point - p.pos).cross(n) #point = pointNull?
                    qrxn = (point - q.pos).cross(n) #point = pointNull?
                    w = prxn * p.angvel - qrxn * q.angvel
                    J = (v.dot(n) + w) * n
                    J = -(1 + e) * J / ((p.massinv) + (q.massinv) + (prxn * prxn) / pI + (qrxn * qrxn) / qI)
                    impulse = Vec2d(J)
                    if p.color == GREEN and pShapeRemoved == False and impulse.length > 5000:
                        pShapeRemoved = True
                        p.color = RED
                        shapesToRemove.append(p)
                        global enemiesDead
                        enemiesDead += 1
                    p.pos += sepVec * p.massinv * mu
                    q.pos -= sepVec * q.massinv * mu
                    p.add_impulse(J, point)
                    q.add_impulse(-J, point)
                    p.update_points
                    q.update_points
                if p.vel.length < sleep_vel and abs(p.angvel) < sleep_angvel:
                    p.sleeping = True
                if q.vel.length < sleep_vel and abs(q.angvel) < sleep_angvel:
                    q.sleeping = True
                    
        for wall in walls:
            max_d = -1e15
            point = Vec2d(0,0)
            for t in p.points:
                d = (wall.pos - t).dot(wall.normal)
                if (d > max_d):
                    max_d = d
                    point = t
                    v = p.vel
                    n = wall.normal
                    w = p.angvel
                    rxn = (point - p.pos).cross(n)
                    I = p.moment
                    J = (-(1 + wall.e) * (v.dot(n) + (w * (rxn))) * n) / (p.massinv + (rxn * rxn) / I)
            if max_d > 0:
                p.sleeping = False
                impulse = Vec2d(J)
                if p.color == GREEN and pShapeRemoved == False and impulse.length > 5000:
                    pShapeRemoved = True
                    p.color = RED
                    shapesToRemove.append(p)
                    global enemiesDead
                    enemiesDead += 1
                p.pos += wall.normal * max_d
                p.add_impulse(J, point)
                p.update_points()
            if p.vel.length < sleep_vel and abs(p.angvel) < sleep_angvel:
                p.sleeping = True
                
    """ (8) Collision detection """
    """ (9) Collision resolution """
    for p in shapes:
        p.update_points()
        p.update_axes()
        p.sleeping = False
        
    remove_shapes(shapesToRemove,shapes, world)
            
    # update any shapes that have been moved
    return False

class Rectangle(Shape):
    def __init__(self, pos, vel, angle, angvel, color, density, length, height):
        self.unscaled_points = [Vec2d(0.5, 0.5), Vec2d(-0.5, 0.5), 
                                Vec2d(-0.5, -0.5), Vec2d(0.5, -0.5)]
        points = []
        for p in self.unscaled_points:
            points.append(Vec2d(p.x*length, p.y*height))
        mass = density*length*height
        moment = (mass/12)*(length*length + height*height)
        super().__init__(pos, vel, angle, angvel, color, mass, moment, points)
        
class Octagon(Shape):
    def __init__(self, pos, vel, angle, angvel, color, density, length, height):
        self.unscaled_points = [Vec2d(0.2, 0.5), Vec2d(-0.2, 0.5), 
                                Vec2d(-0.5, 0.2), Vec2d(-0.5, -0.2),
                                Vec2d(-0.2, -0.5), Vec2d(0.2, -0.5), 
                                Vec2d(0.5, -0.2), Vec2d(0.5, 0.2)]
        points = []
        for p in self.unscaled_points:
            points.append(Vec2d(p.x*length, p.y*height))
        mass = density*length*height
        moment = (mass/12)*(length*length + height*height)
        super().__init__(pos, vel, angle, angvel, color, mass, moment, points)
           
def random_color(minimum, maximum):
    while True:
        color = (random.randrange(256), random.randrange(256), random.randrange(256))
        value = sum(color)
        if value > minimum and value < maximum:
            break
    return color
    
def create_level1(moving, world):
    shape = Rectangle((975,700), (0,0), 0, 0, BLUE, 1, 50, 200)
    world.add(shape)
    shape.add_impulse(Vec2d(0,0), Vec2d(0,0))
    moving.append(shape)
    
    shape2 = Rectangle((1125,700), (0,0), 0, 0, BLUE, 1, 50, 200)
    world.add(shape2)
    shape2.add_impulse(Vec2d(0,0), Vec2d(0,0))
    moving.append(shape2)
    
    shape3 = Rectangle((1050,575), (0,0), 0, 0, BLUE, 1, 200, 50)
    world.add(shape3)
    shape3.add_impulse(Vec2d(0,0), Vec2d(0,0))
    moving.append(shape3)
    
    shape4 = Rectangle((1225,700), (0,0), 0, 0, BLUE, 1, 50, 200)
    world.add(shape4)
    shape4.add_impulse(Vec2d(0,0), Vec2d(0,0))
    moving.append(shape4)
    
    shape5 = Rectangle((1375,700), (0,0), 0, 0, BLUE, 1, 50, 200)
    world.add(shape5)
    shape5.add_impulse(Vec2d(0,0), Vec2d(0,0))
    moving.append(shape5)
    
    shape6 = Rectangle((1300,575), (0,0), 0, 0, BLUE, 1, 200, 50)
    world.add(shape6)
    shape6.add_impulse(Vec2d(0,0), Vec2d(0,0))
    moving.append(shape6)
    
    enemy1 = Octagon((1300,775), (0,0), 0, 0, GREEN, 1, 60, 60)
    world.add(enemy1)
    enemy1.add_impulse(Vec2d(0,0), Vec2d(0,0))
    moving.append(enemy1)
    
    enemy2 = Octagon((1050,775), (0,0), 0, 0, GREEN, 1, 60, 60)
    world.add(enemy2)
    enemy2.add_impulse(Vec2d(0,0), Vec2d(0,0))
    moving.append(enemy2)
    
    #enemy3 = Octagon((800,775), (0,0), 0, 0, GREEN, 1, 60, 60)
    #world.add(enemy3)
    #enemy3.add_impulse(Vec2d(0,0), Vec2d(0,0))
    #moving.append(enemy3)
    
def main():
    pygame.init()
    world = World(1500, 800, WHITE)
    global enemiesDead
    enemiesDead = 0
    level = 1
    lineScreen = pygame.Surface((3, 800))
    lineScreen.fill(BLUE)
    world.screen.blit(lineScreen,(300, 0))
    font = pygame.font.Font(None, 50)
    win_text = font.render("You Win!", 1, (0, 0, 0))
    
    world.display()
    
    moving = []

    clock = pygame.time.Clock()
    done = False
    #density = 1 # mass / area
    #timesteps = 0
    inRange = False;

    create_level1(moving, world)
    
    while not done:
        #print(shape.pos, shape.vel, shape.angle, shape.angvel)
        # Check for events
        for event in pygame.event.get():
            if event.type == pygame.QUIT: # Close window clicked
                done = True
                break
            if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1: # left mouse button
                        if event.pos[0] <= 300:
                            startPos = event.pos
                            inRange = True
                        else:
                            inRange = False
            if event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1: # left mouse button
                        if inRange == True and event.pos[0] <= 300:
                            endPos = event.pos
                            launchVec = Vec2d(startPos) - Vec2d(endPos)
                            shape = Octagon(endPos, (0,0), 0, 0, RED, 1, 80, 80)
                            world.add(shape)
                            shape.add_impulse(Vec2d(0, 1000), endPos + Vec2d(50, 50))
                            if launchVec.length > 300:
                                launchVec = launchVec.normalized() * 300
                            shape.vel = launchVec * 0.05
                            moving.append(shape)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    if level == 1:
                        world = World(1500, 800, WHITE)
                        moving = []
                        global enemiesDead
                        enemiesDead = 0
                        create_level1(moving, world)

        # Velocity Verlet method
        n = 1
        dt = 1 / n
        collide_max = 3
        for i in range(n):        
            update_force(moving, world)
            update_vel(moving, 0.5*dt)
            update_pos(moving, dt)
            collide_count = 0
            update_force(moving, world)
            update_vel(moving, 0.5*dt)
            while collide_count < collide_max and handle_collisions(moving, world):
                collide_count += 1
        #print(shape.pos, shape.angle)
        world.display()
        world.screen.blit(lineScreen,(300, 0))
        if enemiesDead == 2 and level == 1:
            world.screen.blit(win_text,(685,375))
        pygame.display.flip()
        
        clock.tick(60) # wait so that this only updates 60 fps maximum
        
    pygame.quit() # quit nicely, so the program window doesn't hang

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        pygame.quit()
        raise e
           