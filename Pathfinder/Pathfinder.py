import sys
import os
os.environ["PYSDL2_DLL_PATH"] = r"..\SDL2-2.32.2-win32-x64"
import sdl2
import sdl2.ext

import numpy as np
import heapq
import time

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

class Map:
	
    map_wdt : int
    map_hgh : int
    tiles = []

    empty_char = ':'
    obstacle_char = '@'
    start_char = 'S'
    end_char = 'X'
    path_char = '+'

    StartPoint = None
    EndPoint = None

    def __init__(self, width: int, height: int):
        self.map_wdt = width
        self.map_hgh = height
        self.tiles = np.empty((self.map_hgh, self.map_wdt), dtype=str)

    def clear(self):
        for y in range(0,self.map_hgh):
            for x in range(0,self.map_wdt):
                self.tiles[y][x] = self.empty_char

    def generate_start_point(self):
        x = np.random.randint(0, self.map_wdt)
        y = np.random.randint(0, self.map_hgh)
        self.tiles[y][x] = self.start_char
        self.StartPoint = Point(x, y)

    def generate_end_point(self):
        while True:
            x = np.random.randint(0, self.map_wdt)
            y = np.random.randint(0, self.map_hgh)
            if self.tiles[y][x] != self.start_char:
                self.tiles[y][x] = self.end_char
                self.EndPoint = Point(x, y)
                break

    def generate_obstacles(self):
        min_count = int(self.map_wdt * self.map_hgh * 0.05)
        max_count = int(self.map_wdt * self.map_hgh * 0.1)
        count = np.random.randint(min_count, max_count)

        while count > 0:
            x = np.random.randint(0, self.map_wdt-1)
            y = np.random.randint(0, self.map_hgh-1)
            if self.tiles[y][x] not in (self.start_char, self.end_char, self.obstacle_char):
                self.tiles[y][x] = self.obstacle_char
                count -= 1


    def heuristic(self, a: Point, b: Point):
        return abs(a.x - b.x) + abs(a.y - b.y)

    def getNeighbours(self, p: Point):
        neighbours = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = p.x + dx, p.y + dy
            if 0 <= nx < self.map_wdt and 0 <= ny < self.map_hgh:
                if self.tiles[ny][nx] != self.obstacle_char:
                    neighbours.append(Point(nx, ny))
        return neighbours

    def aStar(self, start: Point, goal: Point):
        openSet = []
        heapq.heappush(openSet, (0, start))
        cameFrom = {}
        costSoFar = {}
        cameFrom[start] = None
        costSoFar[start] = 0

        timer_start = time.time()

        while openSet:
            _, current = heapq.heappop(openSet)

            if time.time() - timer_start > 0.1:
                return []

            if current == goal:
                path = []
                while current:
                    path.append(current)
                    current = cameFrom[current]
                path.reverse()
                return path

            for neighbor in self.getNeighbours(current):
                newCost = costSoFar[current] + 1
                if neighbor not in costSoFar or newCost < costSoFar[neighbor]:
                    costSoFar[neighbor] = newCost
                    priority = newCost + self.heuristic(neighbor, goal)
                    heapq.heappush(openSet, (priority, neighbor))
                    cameFrom[neighbor] = current

        return []

    def draw(self, renderer, tile_size : int, path=set()):
        for y in range(self.map_hgh):
            for x in range(self.map_wdt):
                p = Point(x, y)
                if p in path and p!=self.StartPoint and p!=self.EndPoint:
                    renderer.color = sdl2.ext.Color(128,48,48)
                else:
                    tile = self.tiles[y][x]
                
                    if tile == self.empty_char:
                        renderer.color = sdl2.ext.Color(64, 64, 64)
                    
                    if tile == self.obstacle_char:
                        renderer.color = sdl2.ext.Color(32, 32, 32)
                    
                    if tile == self.start_char:
                        renderer.color = sdl2.ext.Color(48, 128, 48)
                    
                    if tile == self.end_char:
                        renderer.color = sdl2.ext.Color(48, 48, 128)
                rect = sdl2.SDL_Rect(tile_size*x, tile_size*y, tile_size, tile_size)
                renderer.fill(rect)

def main():

    WINDOW_COLOR = sdl2.ext.Color(48,48,48)
    WINDOW_SIZE = (800,600)

    tile_size = 8
    width = int(WINDOW_SIZE[0] / tile_size)
    height = int(WINDOW_SIZE[1] / tile_size)

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
        mapa = Map(width, height)
        mapa.clear()
        mapa.generate_start_point()
        mapa.generate_end_point()
        mapa.generate_obstacles()

        path = mapa.aStar(mapa.StartPoint, mapa.EndPoint)

        # render
        renderer.color = WINDOW_COLOR
        renderer.clear()
        mapa.draw(renderer, tile_size, path)
        renderer.present()
		
        del mapa
        time.sleep(1)

if __name__ == '__main__':
	main()