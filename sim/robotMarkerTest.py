import pygame
import sys
import math

pygame.init()

# Set up the display
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Pygame Moving and Rotating Rectangle")

# Rectangle properties
rect_width, rect_height = 100, 50
rect_color = (0, 0, 150)  # Blue
circle_pos = [(500,400),(500,100),(300,200)]
move_speed = 0.1
rotate_speed = 0.05
angle = 0  # Initial rotation angle
move = False
i=0

# Initial position of the rectangle
rect_x, rect_y = width // 2, height // 2

# Create a surface for the rectangle
rect_surface = pygame.Surface((rect_width, rect_height), pygame.SRCALPHA)
rect_surface.fill(rect_color)

def calculate_angle(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    
    radians = math.atan2(-dy, dx)  # Note: y-coordinates are inverted in Pygame
    return math.degrees(radians)




# Main game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Get the current state of the keyboard
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:  # Rotate counterclockwise
        angle += rotate_speed
    if keys[pygame.K_RIGHT]:  # Rotate clockwise
        angle -= rotate_speed
    if keys[pygame.K_UP] or move:  # Move forward in the direction of rotation
        rect_x += move_speed * math.cos(math.radians(angle))
        rect_y -= move_speed * math.sin(math.radians(angle))
        
    
    # Clear the screen
    screen.fill((0, 0, 0))  # White background
    
    # Rotate the rectangle
    rotated_rect_surface = pygame.transform.rotate(rect_surface, angle)
    rect_rect = rotated_rect_surface.get_rect(center=(rect_x, rect_y))
    
    # Draw the rotated rectangle
    screen.blit(rotated_rect_surface, rect_rect.topleft)

    pygame.draw.circle(screen,(255,255,255), circle_pos[i],10)

    error = calculate_angle(rect_x,rect_y, circle_pos[i][0],circle_pos[i][1])
    print(int(rect_y), circle_pos[i][1])
    angle = error
    if (int(rect_x) != circle_pos[i][0] and int(rect_y) != circle_pos[i][1]):
        move = True

    
    Inx = circle_pos[i][0] - rect_x 
    Iny = circle_pos[i][1] - rect_y
    In = math.sqrt(Inx**2+Iny**2)
    #print(In)
    if In<=10:
        i= i+1
        


    # Update the display
    pygame.display.flip()

# Clean up
pygame.quit()
sys.exit()
