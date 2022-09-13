#constants
#Colors
grey = (70, 70,70)
blue = (0, 0, 255)
green = (0, 255,0)
red = (255,0,0)
yellow = (100, 100, 50)
white = (255, 255,255)

#environment
min_obs_size = 10
obs_variance = 50
local_planner_step = 3
edge_discretization_step = 10

#graphics
node_rad = 2
edge_thickness = 2
edge_color = blue
node_color = blue

#dubbins
dubbins_advance_step = 3 #pixels
dubbins_rotation_step = 8 #degs
dubbins_end_distance  = (360/dubbins_rotation_step)*dubbins_advance_step/7

#prm
prm_proximity_radius = 100
prm_num_connections = 5
import pygame
'''utility pygame functions'''
def pygame_events(event_list = []):
    pygame.display.update()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False
        for e in event_list:
            if event.type == e: return  e
    return True
def pygame_wait_for_key():
    while(True):
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                return event.key
            if event.type == pygame.QUIT:
                return pygame.QUIT
def pygame_print_text(canvas, pos, text, size):
    font = pygame.font.Font(pygame.font.get_default_font(), size)
    h=0
    for line in text.splitlines():
        text_surface = font.render(line, True, (0, 0, 0))
        canvas.blit(text_surface,dest=(pos[0],pos[1]+h))
        h+=size*3//2
    #pygame.display.update()
    pygame.display.flip()