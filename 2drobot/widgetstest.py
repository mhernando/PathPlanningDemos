import pygame
import pygame_widgets
from pygame_widgets.button import Button
from pygame_widgets.slider import Slider
from pygame_widgets.textbox import TextBox

pygame.init()
win = pygame.display.set_mode((600, 600))
button = Button(win, 150, 150, 100, 50)
slider = Slider(win, 100, 100, 30, 200, min=0, max=99, step=1, vertical = True)
output = TextBox(win, 475, 200, 50, 50, fontSize=10)
output.disable()
run = True
while run:
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            pygame.quit()
            run = False
            quit()
            
    win.fill((255, 255, 255))
    output.setText(slider.getValue())
    # Now
    pygame_widgets.update(events)
    
    # Instead of
    button.listen(events)
    button.draw()
    
    pygame.display.update()