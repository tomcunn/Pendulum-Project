import pygame
import sys
import math

# Initialize Pygame
pygame.init()
# Clock for controlling the frame rate
clock = pygame.time.Clock()
# Set desired frame rate
fps = 100
# Initialize font
font = pygame.font.SysFont(None, 36)  # Use default font, size 36
# Frame counter
frame_count = 0

#Length of 1 m track in pixels = 760
meters_to_pixels = 760

# Set the dimensions of the window
screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))

#Rod Properties
Rod_mass = 0.032      #32 g
Rod_length = 0.3556   #m
g = 9.81
J = 1/3*Rod_mass*Rod_length*Rod_length
print(J)
cart_accel = 0
angle_velocity = 0


# Set the title of the window
pygame.display.set_caption("Pendulum Simulation")

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0,0,255)
x = 0
# Line properties
base_point = (x, 300)  # Center of the screen
line_length = 150
angle = 1.0  # Initial angle
time_step = 0.01
prev_angle = 0
cart_vel = 0
counter = 0

# Main game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    counter = counter + 1
    #Compute the forces
    sin_theta = math.sin(math.radians(angle))
    cos_theta = math.cos(math.radians(angle))

    # m*l*g*sin(0)
    g_force = (Rod_length/2)*Rod_mass*g*sin_theta
   
    # m*l*cos(0)*a
    cart_force = -1*Rod_mass*(Rod_length/2)*cart_accel*cos_theta
    print(cart_force)

    #Compute acceleration (Convert to deg/sec^2)
    angle_acceleration = (g_force+cart_force)/J*(180/3.14)
    
    #Compute velocity
    angle_velocity = (angle_velocity + (angle_acceleration * time_step))

    #Compute new angle
    angle = angle + (angle_velocity*time_step)

   

    #Calculate a stop point
    if(x>=0.4):
        cart_accel = -1*cart_vel/time_step
        x = 0.4
        cart_vel = 0
    elif(x<=-0.4):
        cart_accel = -1*cart_vel/time_step
        x = -0.4
        cart_vel = 0
    else:
        if(x > 0.005):
            setpoint = -0.1 * x
        elif(x<-0.005):
            setpoint = 0.1 * x
        else:
            setpoint = 0

        cart_vel = cart_vel + cart_accel * time_step
        x = x + cart_vel * time_step
        base_point = (x*meters_to_pixels+400, 300)  # Center of the screen
        
        cart_accel = (setpoint - angle) * -2.0 + (angle-prev_angle) * 8.0
        
        prev_angle = angle



    # Calculate the endpoint of the line using trigonometry
    # There is one frame difference
    end_point = (
        base_point[0] + line_length * sin_theta,
        base_point[1] + line_length * -cos_theta
    )

    # Increment the frame count
    frame_count += 1

    # Get the actual FPS
    actual_fps = clock.get_fps()

    # Render the frame count text
    frame_text = font.render(f"Frame Count: {round(frame_count*time_step,1)}", True, BLACK)
    pos_text = font.render(f"X pos: {round(x*1000,1)} mm", True, BLACK)
    actual_fps_text =  font.render(f"fps: {round(actual_fps,1)}", True, BLACK)
    angle_text =  font.render(f"Angle: {round(angle,1)} deg", True, BLACK)
    cart_vel_text =  font.render(f"Cart Vel: {round(cart_vel,3)} m/s", True, BLACK)
    


    # Fill the screen with a color
    screen.fill(WHITE)

    # Draw the text on the screen
    screen.blit(frame_text, (10, 10))  # Position at (10, 10)
    screen.blit(pos_text, (10, 50))  # Position at (10, 10)
    screen.blit(actual_fps_text, (10, 90))  # Position at (10, 10) 
    screen.blit(angle_text, (400, 10))  # Position at (10, 10) 
    screen.blit(cart_vel_text, (400, 50))  # Position at (10, 10) 
     
    
    # Draw the rotating line
    pygame.draw.line(screen, BLUE, (20,300), (780,300),20)    
    pygame.draw.line(screen, BLACK, base_point, end_point, 7)
    

    # Update the display
    pygame.display.flip()

    # Control the frame rate
    clock.tick(fps)

# Quit Pygame
pygame.quit()
sys.exit()