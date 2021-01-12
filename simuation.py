# File to Set the Graphic Interface

import pygame

# pylint: disable=no-member
pygame.init()
# pylint: enable=no-member

screen = pygame.display.set_mode((800,600))

running = True
while(running):
     for event in pygame.event.get():
         # pylint: disable=no-member
         if event.type == pygame.QUIT:
        # pylint: enable=no-member
             running = False