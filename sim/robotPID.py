
import pygame
import math

from requests import head

class Envir:
    def __init__(self, dimentions):
        self.black  = (0, 0, 0)
        self.white  = (255, 255, 255)
        self.green  = (0, 255, 0)
        self.blue   = (0, 0, 255)
        self.red    = (255,0 ,0)
        self.lightblue = (0, 200, 255)

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
    def draw_marker(self, position, color):
        pygame.draw.circle(self.map, color, position, 5)

    def draw_path(self, path):
        if len(path) > 1:
            pygame.draw.lines(self.map, self.green, False, path, 2)


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
    
    def get_nearest_point(self, markers):
        Iout = 0

        Inx = markers[0][0] - self.x 
        Iny = markers[0][1] - self.y
        In = math.sqrt(Inx**2+Iny**2)
        lastIn = In
        
        for p in range(len(markers)):
            Inx = markers[p][0] - self.x 
            Iny = markers[p][1] - self.y
            In = math.sqrt(Inx**2+Iny**2)
            
            if In < lastIn:
                lastIn = In
                Iout = p
        return Iout
    
###PROG###
pygame.init()
start=(500,600)
dims=(800, 1600)
running= True

environment= Envir(dims)
robot = Robot(start,"differentialR.png",0.01*3779.52)

dt = 1e-6
lasttime= pygame.time.get_ticks()

prevX, prevY = robot.get_position()

#Random markerid testimiseks
marker = [[400, 300],[500,200],[700,200],[700,100],[850,100],[850,200],
          [1000,200],[1100,250],[1175,275],[1200,350],[1175,425],[1100,450],
          [1000, 450],[900,500],[800,400],[700,500],[600,400],[500,500],
          [400,500]]


path = [(start[0], start[1])]
#Loendur
i=0
i = robot.get_nearest_point(marker)

#PID 
Kp= 0.5
Ki = 0.01
Kd = 0.01

integral = 0.0
previous_error = 0.0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running=False
        robot.move(event)
    if i == len(marker):
        i = 0
    #Delta time arvutus    
    current_time = pygame.time.get_ticks()
    dt = (current_time-lasttime)/1000
    if dt == 0:
        dt = 1e-6
    lasttime = current_time
    
    pygame.display.update()
    environment.map.fill(environment.black)
    robot.draw(environment.map)
    environment.writeinfo(int(robot.vl),int(robot.vr), robot.theta)

    

    for x in marker:
        environment.draw_marker(x, environment.lightblue)
    environment.draw_marker(marker[i],environment.red)
    
    x, y = robot.get_position()
    path.append((x, y))
    environment.draw_path(path)
    
    #Leiame vektori kui eelmine ja hetke punkt pole samad
    if x != prevX or y != prevY:

        
        heading =robot.calculate_heading(prevX, prevY, x, y)
        prevX , prevY = x , y
        marker_heading = robot.calculate_heading(x, y ,marker[i][0],marker[i][1])
        error = robot.calculate_error_angle(heading,marker_heading)
        #print(heading)
        
        #PID 
        integral += error * dt
        integral = max(min(integral, 100), -100)
        derivative = (error - previous_error) / dt
        previous_error = error
        control_signal = Kp * error + Ki * integral + Kd * derivative
        control_signal = max(min(control_signal, 100), -100)

        #Vel out
        base_speed = 50.0  #Algkiirus
        left_speed = base_speed + control_signal
        right_speed = base_speed - control_signal

        #Kiiruse piiramine
        max_speed = 100
        left_speed = max(min(left_speed, max_speed), -max_speed)
        right_speed = max(min(right_speed, max_speed), -max_speed)
        robot.vel_out(left_speed, right_speed)
        
        #print(left_speed,right_speed)
    
    
    ### Marker on thresholdis ###
    Inx = marker[i][0] - x 
    Iny = marker[i][1] - y
    In = math.sqrt(Inx**2+Iny**2)
    #print(In)
    if In<=10:
        i= i+1

   
