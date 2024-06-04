
import pygame
import math
import numpy as np
from scipy.optimize import minimize

class Envir:
    def __init__(self, dimentions):
        self.black  = (0, 0, 0)
        self.white  = (255, 255, 255)
        self.green  = (0, 255, 0)
        self.blue   = (0, 0, 255)
        self.red    = (255,0 ,0)
        self.yellow = (255, 255, 0)

        self.height = dimentions[0]
        self.width = dimentions[1]

        pygame.display.set_caption('Differential drive robot')
        self.map = pygame.display.set_mode((self.width,
                                            self.height))
        self.font = pygame .font.Font("freesansbold.ttf", 50)
        self.text = self.font.render("default", True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (dimentions[1]- 600, dimentions[0]- 100)


    def writeinfo(self, Vl, Vr, theta):
        txt= f"Vl = {Vl} Vr = {Vr} theta = {int(math.degrees(theta))}"
        self.text = self.font .render(txt, True, self.white, self.black)
        self.map.blit(self.text,self.textRect)
    

    #Markerid
    def draw_marker(self, position):
        pygame.draw.circle(self.map, self.red, position, 10)


class Robot:
    def __init__(self,startpos, robotImg, width):
        self.m2p = 3779.52
        self.w= width
        self.x = startpos[0]
        self.y = startpos[1]
        self.theta = 0
        self.vl = 0.00 
        self.vr = 0.00 
        
        self.img = pygame.image.load(robotImg)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
    
    def draw(self,map):
        map.blit(self.rotated,self.rect)
    #keyboard control
    def move(self, event=None,):
        if event is not None:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    self.vl += 1.0#*self.m2p
                elif event.key == pygame.K_UP:
                    self.vl -= 1.0#*self.m2p
                elif event.key == pygame.K_RIGHT:
                    self.vr += 1.0#*self.m2p
                elif event.key == pygame.K_DOWN:
                    self.vr -= 1.0#*self.m2p
        
        self.x += ((self.vl + self.vr)/2) * math.cos(self.theta)*dt
        self.y -= ((self.vl + self.vr)/2) * math.sin(self.theta)*dt
        self.theta += (self.vr - self.vl)/self.w*dt

        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta),1)
        self.rect = self.rotated.get_rect(center= (self.x, self.y))
    #Vel out, mis ss laheb arduinosse
    def vel_out(self, vl, vr):
        ##############
        self.vl = vl##
        self.vr = vr##
        ##############  
        self.x += ((self.vl + self.vr)/2) * math.cos(self.theta)*dt
        self.y -= ((self.vl + self.vr)/2) * math.sin(self.theta)*dt
        self.theta += (self.vr - self.vl)/self.w*dt
        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.theta),1)
        self.rect = self.rotated.get_rect(center= (self.x, self.y))


    def get_position(self):
        return self.x, self.y
    

    #Vektori arvutus
    def calculate_heading(self, x1, y1, x2, y2):
        delta_x = x2 - x1
        delta_y = y2 - y1
        bearing = math.atan2(delta_y, delta_x)
        bearing = math.degrees(bearing)
        heading = (bearing + 360) % 360
        return heading
    
    #Nurga errori arvutus
    def calculate_error_angle(self, current_heading, target_bearing):
        error_angle = (target_bearing - current_heading + 360) % 360
        if error_angle > 180:
            error_angle -= 360
        return error_angle
    

def mpc_controller(robot, marker, dt):
    def robot_model(state, u, dt):
        x, y, theta = state
        vl, vr = u
        w = robot.w
        x += ((vl + vr) / 2) * math.cos(theta) * dt
        y -= ((vl + vr) / 2) * math.sin(theta) * dt
        theta += (vr - vl) / w * dt
        return x, y, theta

    def cost_fn(u):
        state = [robot.x, robot.y, robot.theta]
    
        cost = 0
        
        for k in range(horizon):
            state = robot_model(state, u[k*2:k*2+2], dt)
            cost += (state[0] - marker[0])**2 + (state[1] - marker[1])**2
        return cost

    horizon = 5
    u0 = [robot.vl, robot.vr] * horizon
    
    bounds = [(-1000, 1000)] * horizon * 2
    res = minimize(cost_fn, u0, bounds=bounds)
    print(res.x[:2])
    return res.x[:2]

###PROG###
pygame.init()
start=(200,200)
dims=(800, 1600)
running= True

environment= Envir(dims)
robot = Robot(start,"differentialR.png",0.01*3779.52)

dt = 0.1
lasttime= pygame.time.get_ticks()

prevX, prevY = robot.get_position()

#Random markerid testimiseks
marker = [[400, 200],[600,100],[800,200],[900,100],[1000,200],[1200,300],[1400,400],
          [1400, 600],[1200,700],[1000,800],[800,800],[600,600],[400,400],[200,400]]


#Loendur
i=0

#PID 
Kp= 0.8
Ki = 0.01
Kd = 0.05

integral = 0.0
previous_error = 0.0

while running:
    for x in marker:
        environment.draw_marker(x)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running=False
        robot.move(event)

    #Delta time arvutus    
    current_time = pygame.time.get_ticks()
    dt = (current_time-lasttime)/1000
    #if dt == 0:
        #dt = 1e-6
    lasttime = current_time
    
    pygame.display.update()
    environment.map.fill(environment.black)
    robot.draw(environment.map)
    environment.writeinfo(int(robot.vl),int(robot.vr), robot.theta)
    
    environment.draw_marker(marker[i])

    x, y = robot.get_position()
    #Leiame vektori kui eelmine ja hetke punkt pole samad
    if x != prevX or y != prevY:

        heading =robot.calculate_heading(prevX, prevY, x, y)
        prevX , prevY = x , y
        marker_heading = robot.calculate_heading(x, y ,marker[i][0],marker[i][1])
        error = robot.calculate_error_angle(heading,marker_heading)

        base_speed = 100.0  #Algkiirus
        #PID 
        #integral += error * dt
        #derivative = (error - previous_error) / dt
        #previous_error = error
        
        #control_signal = Kp * error + Ki * integral + Kd * derivative
        
        vl, vr = mpc_controller(robot, marker[i], dt)
        
        #Vel out
        #left_speed = base_speed + control_signal
        #right_speed = base_speed - control_signal
        #Kiiruse piiramine
        #max_speed = 5 
        #left_speed = max(min(left_speed, max_speed), -max_speed)
        #right_speed = max(min(right_speed, max_speed), -max_speed)
       
        robot.vel_out(vl/10, vr/10)
    
    
    ### Marker on thresholdis ###
    Inx = marker[i][0] - x 
    Iny = marker[i][1] - y
    In = math.sqrt(Inx**2+Iny**2)
    #print(In)
    if In<=20:
        i= i+1
    pygame.time.delay(20)
   

