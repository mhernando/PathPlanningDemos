import pygame
import pygame_widgets

pygame.init()
win = pygame.display.set_mode((1200, 600))

run = True
while run:
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            pygame.quit()
            run = False
            quit()
            
    win.fill((255, 255, 255))

    pygame_widgets.update(events)
    
    # Instead of
   
    
    pygame.display.update()