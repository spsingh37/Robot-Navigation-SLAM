import pygame
import pygame.freetype
import math
import sys
import numpy as np
from colors import *

map_height = int(sys.argv[2])
map_width = int(sys.argv[1])
cell_size = int(50*(10/map_height))

map = np.zeros((map_width,map_height))
save_path = "test_out.map"

pygame.init()
window = pygame.display.set_mode((cell_size*map_width, cell_size*map_height))
window.fill(grey)
font = pygame.freetype.SysFont("Ubuntu Mono Bold", cell_size/4)
font_large = pygame.freetype.SysFont("Ubuntu Mono Bold", cell_size/2)

def initialize_map():
        for j in range(map_height):
            for i in range(map_width):
                if(map[i][j] == 0):
                    pygame.draw.rect(window, white, (i*cell_size,j*cell_size,cell_size,cell_size),0)
                    pygame.draw.rect(window, black, (i*cell_size,j*cell_size,cell_size,cell_size),2)
mode = 1
initialize_map()
while(True):
    pygame.display.update()
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LSHIFT:
                mode = 0
            elif event.key == pygame.K_s:
                mode = 2
            elif event.key == pygame.K_g:
                mode = 3
            elif event.key == pygame.K_m:
                np.savetxt(save_path, map, delimiter=',', fmt='%d')
                print("maped saved to: " + save_path)
        if event.type == pygame.KEYUP:
            mode = 1
        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            cell_x = math.floor(pos[0]/cell_size)
            cell_y = math.floor(pos[1]/cell_size)
            if mode == 0:
                map[cell_x][cell_y] = 0
                pygame.draw.rect(window, white, (cell_x*cell_size,cell_y*cell_size,cell_size,cell_size),0)
                pygame.draw.rect(window, black, (cell_x*cell_size,cell_y*cell_size,cell_size,cell_size),2)
            elif mode == 2:
                map[cell_x][cell_y] = 2
                pygame.draw.rect(window, blue, (cell_x*cell_size,cell_y*cell_size,cell_size,cell_size),0)
                pygame.draw.rect(window, black, (cell_x*cell_size,cell_y*cell_size,cell_size,cell_size),2)
            elif mode == 3:
                map[cell_x][cell_y] = 3
                pygame.draw.rect(window, red, (cell_x*cell_size,cell_y*cell_size,cell_size,cell_size),0)
                pygame.draw.rect(window, black, (cell_x*cell_size,cell_y*cell_size,cell_size,cell_size),2)
            elif mode == 1: 
                map[cell_x][cell_y] = 1
                pygame.draw.rect(window, black, (cell_x*cell_size,cell_y*cell_size,cell_size,cell_size),0)
                pygame.draw.rect(window, black, (cell_x*cell_size,cell_y*cell_size,cell_size,cell_size),2)
        if event.type == pygame.QUIT:
            pygame.quit() 
            sys.exit()
