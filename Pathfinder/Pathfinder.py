import sys
import os
os.environ["PYSDL2_DLL_PATH"] = r"..\SDL2-2.32.2-win32-x64"
import sdl2
import sdl2.ext

import numpy as np
import heapq
import time

WINDOW_COLOR = sdl2.ext.Color(48, 48, 48)
WINDOW_SIZE = (600, 480)

tile_size = 8

map_wdt = int(WINDOW_SIZE[0]/tile_size)
map_hgh = int(WINDOW_SIZE[1]/tile_size)
mapa = np.empty((map_hgh, map_wdt), dtype=str)

empty_char = ':'
obstacle_char = '@'
start_char = 'S'
end_char = 'X'
path_char = '+'

##############################

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return isinstance(other, Point) and self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)

    def __repr__(self):
        return f"({self.x},{self.y})"

StartPoint = None
EndPoint = None

##############################

def clear_map():
    for y in range(map_hgh):
        for x in range(map_wdt):
            mapa[y][x] = empty_char

def generate_start_point():
    global StartPoint
    x = np.random.randint(0, map_wdt)
    y = np.random.randint(0, map_hgh)
    mapa[y][x] = start_char
    StartPoint = Point(x, y)

def generate_end_point():
    global EndPoint
    while True:
        x = np.random.randint(0, map_wdt)
        y = np.random.randint(0, map_hgh)
        if mapa[y][x] != start_char:
            mapa[y][x] = end_char
            EndPoint = Point(x, y)
            break

def generate_obstacles():
    min_count = int(map_wdt * map_hgh * 0.05)
    max_count = int(map_wdt * map_hgh * 0.1)
    count = np.random.randint(min_count, max_count)
    while count > 0:
        x = np.random.randint(0, map_wdt-1)
        y = np.random.randint(0, map_hgh-1)
        if mapa[y][x] not in (start_char, end_char, obstacle_char):
            mapa[y][x] = obstacle_char
            count -= 1

##############################

def heuristic(a: Point, b: Point):
    return abs(a.x - b.x) + abs(a.y - b.y)

def getNeighbours(p: Point):
    neighbours = []
    for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
        nx, ny = p.x + dx, p.y + dy
        if 0 <= nx < map_wdt and 0 <= ny < map_hgh:
            if mapa[ny][nx] != obstacle_char:
                neighbours.append(Point(nx, ny))
    return neighbours

def aStar(start: Point, goal: Point):
    openSet = []
    heapq.heappush(openSet, (0, start))
    cameFrom = {}
    costSoFar = {}
    cameFrom[start] = None
    costSoFar[start] = 0

    while openSet:
        _, current = heapq.heappop(openSet)

        if current == goal:
            path = []
            while current:
                path.append(current)
                current = cameFrom[current]
            path.reverse()
            return path

        for neighbor in getNeighbours(current):
            newCost = costSoFar[current] + 1
            if neighbor not in costSoFar or newCost < costSoFar[neighbor]:
                costSoFar[neighbor] = newCost
                priority = newCost + heuristic(neighbor, goal)
                heapq.heappush(openSet, (priority, neighbor))
                cameFrom[neighbor] = current

    return []

##############################

def draw_map(renderer, path=set()):
    for y in range(map_hgh):
        for x in range(map_wdt):
            p = Point(x, y)
            if p in path and p!=StartPoint and p!=EndPoint:
                renderer.color = sdl2.ext.Color(128,48,48)
            else:
                tile = mapa[y][x]
                
                if tile == empty_char:
                    renderer.color = sdl2.ext.Color(64, 64, 64)
                    
                if tile == obstacle_char:
                    renderer.color = sdl2.ext.Color(32, 32, 32)
                    
                if tile == start_char:
                    renderer.color = sdl2.ext.Color(48, 128, 48)
                    
                if tile == end_char:
                    renderer.color = sdl2.ext.Color(48, 48, 128)
            rect = sdl2.SDL_Rect(tile_size*x, tile_size*y, tile_size, tile_size)
            renderer.fill(rect)

##############################

sdl2.ext.init()
window = sdl2.ext.Window("Pathfinder", WINDOW_SIZE)
window.show()

renderer = sdl2.ext.Renderer(window)

while True:
    
    # handle events
    events = sdl2.ext.get_events()
    for event in events:
        if event.type == sdl2.SDL_QUIT:
            sdl2.ext.quit()
            sys.exit(0)
    
    # update
    clear_map()
    generate_start_point()
    generate_end_point()
    generate_obstacles()
    path = aStar(StartPoint, EndPoint)
    
    # render
    renderer.color = WINDOW_COLOR
    renderer.clear()
    draw_map(renderer, set(path))
    renderer.present()
    
    time.sleep(1)

sdl2.ext.quit()
sys.exit(0)