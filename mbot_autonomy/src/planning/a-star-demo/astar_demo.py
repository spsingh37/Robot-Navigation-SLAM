import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
import pygame.freetype
import math
import sys
import numpy as np
from colors import *
from astar import *

def print_usage_and_exit():
    sys.stderr.write("usage: astar_demo.py <mapfile> <algorithm> <n-ways>\n")
    sys.stderr.write("<algorithm>: \n\t-a : astar\n\t-g : greedy\n\t-d : dijkstras\n")
    sys.stderr.write("<n-ways>: \n\t4 : four-way search\n\t8 : eight-way search\n")
    sys.exit(1)


if len(sys.argv) != 4:
    print("\nERROR: invalid arguments\n")
    print_usage_and_exit()

filename = str(sys.argv[1])
algorithm = str(sys.argv[2])
nways = int(sys.argv[3])
alg = NONE
if(algorithm == '-a'):
    alg = ASTAR
if(algorithm == '-g'):
    alg = GREEDY
if(algorithm == '-d'):
    alg = DIJKST

if(alg == NONE):
    print("\nERROR: invalid argument\n")
    print_usage_and_exit()
if((nways!=4) & (nways!=8)):
    print("\nERROR: invalid argument\n")
    print_usage_and_exit()
    
map = np.loadtxt(filename, delimiter=",", dtype=int)
map_width = map.shape[0]
map_height = map.shape[1]
cell_size = 100
cell_size = int(50*(10/map_height))
(start_x, start_y) = extract_start(map)
(goal_x, goal_y) = extract_goal(map)


pygame.init()
window = pygame.display.set_mode((cell_size*map_width, cell_size*map_height))
window.fill(grey)
pygame.key.set_repeat(1, 100)
font = pygame.freetype.SysFont("Ubuntu Mono Bold", cell_size/4)
font_large = pygame.freetype.SysFont("Ubuntu Mono Bold", cell_size/2)

def draw_scores(cell_row, cell_col, g, h, f):
    if(alg == ASTAR):
        if(cell_size > 25):
            font.render_to(window,(cell_row*cell_size + cell_size/20, cell_col*cell_size + cell_size/10),str(g),black)
            font.render_to(window,(cell_row*cell_size + cell_size - cell_size/3, cell_col*cell_size + cell_size/10),str(h),black)
        if(f<100):
            font_large.render_to(window,(cell_row*cell_size + cell_size/4, cell_col*cell_size + cell_size/2),str(f),black)
        if(f>=100):
            font_large.render_to(window,(cell_row*cell_size + cell_size/8, cell_col*cell_size + cell_size/2),str(f),black)
    if(alg == DIJKST):
        if(g<100):
            font_large.render_to(window,(cell_row*cell_size + cell_size/4, cell_col*cell_size + cell_size/2),str(g),black)
        if(g>=100):
            font_large.render_to(window,(cell_row*cell_size + cell_size/8, cell_col*cell_size + cell_size/2),str(g),black)
    if(alg == GREEDY):
        if(h<100):
            font_large.render_to(window,(cell_row*cell_size + cell_size/4, cell_col*cell_size + cell_size/2),str(h),black)
        if(h>=100):
            font_large.render_to(window,(cell_row*cell_size + cell_size/8, cell_col*cell_size + cell_size/2),str(h),black)

def draw_letter(cell_row, cell_col, letter):
    font_large.render_to(window,(cell_row*cell_size + cell_size/3, cell_col*cell_size + cell_size/3),letter,black)

def initialize_map():
        for j in range(map_height):
            for i in range(map_width):
                if(map[i][j] == 0):
                    pygame.draw.rect(window, white, (i*cell_size,j*cell_size,cell_size,cell_size),0)
                    pygame.draw.rect(window, black, (i*cell_size,j*cell_size,cell_size,cell_size),2)
                if(map[i][j] == 1):
                    pygame.draw.rect(window, black, (i*cell_size,j*cell_size,cell_size,cell_size),0)
                    pygame.draw.rect(window, black, (i*cell_size,j*cell_size,cell_size,cell_size),2)
                if(map[i][j] == 2):
                    pygame.draw.rect(window, blue, (i*cell_size,j*cell_size,cell_size,cell_size),0)
                    pygame.draw.rect(window, black, (i*cell_size,j*cell_size,cell_size,cell_size),2)
                    draw_letter(i,j,'S')
                if(map[i][j] == 3):
                    pygame.draw.rect(window, blue, (i*cell_size,j*cell_size,cell_size,cell_size),0)
                    pygame.draw.rect(window, black, (i*cell_size,j*cell_size,cell_size,cell_size),2)
                    draw_letter(i,j,'G')

def update_map(astar):
    for node in astar.searched_list.nodes:
        if((node != astar.start)&(node != astar.goal)):
            pygame.draw.rect(window, green, (node.x*cell_size,node.y*cell_size,cell_size,cell_size),0)
            pygame.draw.rect(window, black, (node.x*cell_size,node.y*cell_size,cell_size,cell_size),1)
            draw_scores(node.x, node.y, node.g_cost, node.h_cost, node.f_cost())
    for node in astar.closed_list.nodes:
        if((node != astar.start)&(node != astar.goal)):
            pygame.draw.rect(window, red, (node.x*cell_size,node.y*cell_size,cell_size,cell_size),0)
            pygame.draw.rect(window, black, (node.x*cell_size,node.y*cell_size,cell_size,cell_size),1)
            draw_scores(node.x, node.y, node.g_cost, node.h_cost, node.f_cost())


def render_path(astar):
    for node in astar.path.nodes:
        if((node != astar.start)&(node != astar.goal)):
            pygame.draw.rect(window, blue, (node.x*cell_size,node.y*cell_size,cell_size,cell_size),0)
            pygame.draw.rect(window, black, (node.x*cell_size,node.y*cell_size,cell_size,cell_size),1)
            draw_scores(node.x, node.y, node.g_cost, node.h_cost, node.f_cost())

initialize_map()

astar = AstarSearch(start_x,start_y,goal_x,goal_y,map)

while(True):
    pygame.display.update()
    for event in pygame.event.get():
        if event.type == pygame.MOUSEBUTTONUP:
            if(not astar.path_found):
                astar.step_path(alg,nways)
                update_map(astar)
            if(astar.path_found):
                render_path(astar)
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_z:
                if(not astar.path_found):
                    astar.step_path(alg,nways)
                    update_map(astar)
                if(astar.path_found):
                    render_path(astar)
        if event.type == pygame.QUIT:
            pygame.quit() 
            sys.exit()
