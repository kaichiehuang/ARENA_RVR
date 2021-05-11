import sys
import os
from sphero_sdk import DriveFlagsBitmask
from sphero_sdk import SerialAsyncDal
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import RvrStreamingServices
from sphero_sdk import RawMotorModesEnum
import asyncio
import heapq
import time
from arena import *
import RPi.GPIO as GPIO
import numpy as np
import math
from collections import deque
sys.path.append(os.path.abspath(os.path.join(
    os.path.dirname(__file__), '../../../')))
scene = Scene(host="arena.andrew.cmu.edu", realm="realm", scene="jameshuang")
loop, rvr = None, None
distance_to_obstacle = 10000  # in cm
rvr_direction = (0, 1)
rvr_position = (0, 0)  # Data from locator
grid = None  # grid map
canvas = {}
RED = (255, 0, 0)
destinations = {}
HUD_distance = None

# setting up rpi GPIO
GPIO.setmode(GPIO.BCM)
trigger = 20
echo = 21
GPIO.setup(trigger, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)


class GridMap:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = []
        self.draw_outline()

    def in_bounds(self, id):
        (x, y) = id
        return -1*self.width/2 < x < self.width/2 and -1*self.height/2 <= y < self.height/2

    def passable(self, id):
        return id not in self.obstacles

    def neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        if (x + y) % 2 == 0:
            results.reverse()  # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

    def add_obstacle(self, id):
        self.obstacles.append(id)

    def draw_outline(self):
        # height_arena = self.height * 0.3
        # width_arena = self.width * 0.3
        height_arena = 5 * 0.3
        width_arena = 5 * 0.3
        start = (-0.15, 0, 0.15)
        for _ in range(5+1):
            draw_line(start, tuple(np.add(start, (width_arena, 0, 0))))
            start = tuple(np.add(start, (0, 0, -0.3)))

        start = (-0.15, 0, 0.15)
        for _ in range(5+1):
            draw_line(start, tuple(np.add(start, (0, 0, -height_arena))))
            start = tuple(np.add(start, (0.3, 0, 0)))


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def plan_path(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    came_from[start] = None
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            break
        for next in graph.neighbors(current):
            if next not in came_from:
                priority = heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    return came_from


def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    print("in reconstruct path, goal is {}".format(goal))
    print(came_from)
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path


def pose_convert(original, to, position):
    if original == 'arena' and to == 'grid':
        x = round(position[0]/0.3)
        y = -round(position[2]/0.3)
        print("After convert, ({},{})".format(x, y))
        return (x, y)
    elif original == 'grid' and to == 'arena':
        x = position[0] * 0.3
        y = 0
        z = -position[1] * 0.3
        print("After convert, ({},{},{})".format(x, y, z))
        return (x, y, z)
    elif original == 'rvr' and to == 'grid':
        x = round(position[0]/0.3)
        y = round(position[1]/0.3) + 1
        print("After convert, ({},{})".format(x, y))
        return (x, y)
    elif original == 'grid' and to == 'rvr':
        x = position[0] * 0.3
        y = (position[1]-1) * 0.3
        print("After convert, ({},{})".format(x, y))
        return (x, y)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'
    Source: https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python#answer-13849249 """
    angle = np.math.atan2(np.linalg.det([v1, v2]), np.dot(v1, v2))
    return np.degrees(angle)


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

# below are rvr functions


async def locator_handler(locator_data):
    global rvr_position
    # print('Locator data response: ', locator_data)
    loc_dict = locator_data['Locator']
    if loc_dict['is_valid']:
        # print(loc_dict['X'])
        # print(loc_dict['Y'])
        loc_X = loc_dict['X']
        loc_Y = loc_dict['Y']
        rvr_position = (loc_X, loc_Y)


async def color_detected_handler(color_detected_data):
    # print('Color detection data response: ', color_detected_data)
    pass


async def rvr_setup():
    global rvr
    await rvr.wake()
    # Give RVR time to wake up
    await asyncio.sleep(2)
    # Reset the yaw and locator.
    await rvr.reset_yaw()
    await asyncio.sleep(.1)
    await rvr.reset_locator_x_and_y()
    await asyncio.sleep(.1)

    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.locator,
        handler=locator_handler
    )
    await rvr.enable_color_detection(is_enabled=True)
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.color_detection,
        handler=color_detected_handler
    )
    await rvr.sensor_control.start(interval=200)


async def rvr_turn(degree):
    global rvr
    print("in turn")
    print(degree)
    await rvr.reset_yaw()
    await rvr.drive_with_heading(
        speed=0,  # Valid speed values are 0-255
        heading=int(degree),  # Valid heading values are 0-359
        flags=DriveFlagsBitmask.none.value
    )
    # Delay to allow RVR to turn
    await asyncio.sleep(1)


async def rvr_drive_straight(turn_point):
    global rvr, distance_to_obstacle, rvr_position
    await rvr.reset_yaw()
    print("distance to obstable is "+str(distance_to_obstacle))
    # keep driving until arrive at turn point or too close to obstacle
    print("Turn point, ({},{})".format(turn_point[0], turn_point[1]))

    while not rvr_has_arrived(pose_convert('grid', 'rvr', turn_point), rvr_position) and distance_to_obstacle >= 8:
        print("Rvr position, ({},{})".format(rvr_position[0], rvr_position[1]))
        await rvr.drive_with_heading(
            speed=20,  # Valid speed values are 0-255
            heading=0,  # Valid heading values are 0-359
            flags=DriveFlagsBitmask.none.value
        )
        erase_waypoint(pose_convert('rvr', 'grid', rvr_position))
        # Delay to allow RVR to drive
        await asyncio.sleep(0.1)
    print('out of while')
    # stop rvr
    # await rvr.raw_motors(left_speed=0, left_mode=0, right_speed=0, right_mode=0)
    await rvr.drive_with_heading(
        speed=0,  # Valid speed values are 0-255
        heading=0,  # Valid heading values are 0-359
        flags=DriveFlagsBitmask.none.value
    )
    print('stopped')
    return rvr_has_arrived(pose_convert('grid', 'rvr', turn_point), rvr_position)


def rvr_has_arrived(p1, p2):
    if abs(p1[0]-p2[0]) < 0.05 and abs(p1[1]-p2[1]) < 0.05:
        print("arrived at turn point")
        return True
    else:
        print("not arrived at turn point")
        return False


async def navigate(start, goal):
    global rvr_direction, rvr_position, rvr, grid

    # start path planning
    came_from = plan_path(grid, start, goal)
    path = reconstruct_path(came_from, start, goal)
    consecutive_vector = []
    print(path)
    draw_path(path[1:], goal)

    # create consecutive path
    seg_start = path[0]
    curr_position = path[0]
    curr_unit_vector = tuple(np.subtract(path[1], path[0]))
    for next_position in path[1:]:
        vector = tuple(np.subtract(next_position, curr_position))
        angle = angle_between(curr_unit_vector, vector)
        if angle != 0:
            direction_vector = tuple(np.subtract(curr_position, seg_start))
            consecutive_vector.append(direction_vector)
            seg_start = curr_position
            curr_unit_vector = tuple(np.subtract(next_position, curr_position))

        curr_position = next_position
    direction_vector = tuple(np.subtract(curr_position, seg_start))
    consecutive_vector.append(direction_vector)

    print(consecutive_vector)
    # start navigation
    for cv in consecutive_vector:
        # angle=angle_between(rvr_direction, cv)
        angle = (angle_between(cv, rvr_direction) + 360) % 360
        print(angle)
        # turn if current direction is not aligned with the vector
        if angle != 0:
            await rvr_turn(angle)
            rvr_direction = unit_vector(cv)

        turn_point = tuple(
            np.add(pose_convert("rvr", "grid", rvr_position), cv))
        drive_result = await rvr_drive_straight(turn_point)
        print(drive_result)

        if not drive_result:  # If doesn't reach the next turn point
            # add obstacle and replan
            obs_location = tuple(
                np.add(pose_convert("rvr", "grid", rvr_position), unit_vector(cv)))
            print("obstacle location is ")
            print(str(obs_location))
            grid.add_obstacle(obs_location)
            draw_obstacle(obs_location)
            erase_path(path)
            # erase future waypoints
            await navigate(pose_convert("rvr", "grid", rvr_position), goal)
            return
    # if get out of for loop, this means arrived
    # turn destination color back
    destinations[goal].update_attributes(color=(255, 255, 255))
    scene.update_object(destinations[goal])
    return


@ scene.run_forever(interval_ms=50)
def update_distance():
    global distance_to_obstacle, HUD_distance
    GPIO.output(trigger, True)
    time.sleep(0.00001)
    GPIO.output(trigger, False)
    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(echo) == 0:
        start_time = time.time()

    while GPIO.input(echo) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time

    distance_to_obstacle = (time_elapsed * 34300) / 2
    
    if HUD_distance is not None:
        HUD_distance.update_attributes(text = str(round(distance_to_obstacle, 1))+' cm')
        scene.update_object(HUD_distance)
    
    

# below are arena functions


def init():
    global destinations
    dest = (3, 3)
    print("after convert "+str(pose_convert("grid", "arena", dest)))
    c = Circle(object_id="my_test",
               position=(0.9, 0, -0.9),
               rotation=(-0.7, 0, 0, 0.7),
               scale=(0.1, 0.1, 0.1),
               color=(255, 255, 255),
               click_listener=True,
               evt_handler=click
               )
    scene.add_object(c)
    destinations[dest] = c


def click(scene, evt, msg):
    global rvr, rvr_position, destinations
    pose = (evt.data.position['x'],
            evt.data.position['y'], evt.data.position['z'])
    if evt.type == "mouseup":
        print("clicked position is"+str(evt.data.position))
        goal = pose_convert('arena', 'grid', pose)
        start = pose_convert('rvr', 'grid', rvr_position)
        asyncio.create_task(navigate(start, goal))
    if evt.type == "mousedown":
        print('in mouse down')
        key = pose_convert('arena', 'grid', pose)
        destinations[key].update_attributes(color=RED)
        scene.update_object(destinations[key])


def draw_line(start, end):
    thickline = ThickLine(
        object_id="line_"+str(start)+str(end),
        lineWidth=10,
        path=(start, end),
        color=(0, 255, 0)
    )
    scene.add_object(thickline)


def draw_path(path, goal):
    global canvas
    print("in draw path")
    print(path)
    for waypoint in path:
        if waypoint != goal:
            print('way point is at ')
            print(waypoint)
            canvas[waypoint] = Circle(
                position=pose_convert('grid', 'arena', waypoint),
                rotation=(-0.7, 0, 0, 0.7),
                scale=(0.07, 0.07, 0.07),
                color=(0, 255, 255)
            )
            scene.add_object(canvas[waypoint])


def erase_path(path):
    global canvas
    for waypoint in path:
        if waypoint in canvas:
            scene.delete_object(canvas[waypoint])
            canvas.pop(waypoint, None)


def erase_waypoint(waypoint):
    global canvas
    if waypoint in canvas:
        scene.delete_object(canvas[waypoint])
        canvas.pop(waypoint, None)


def draw_obstacle(waypoint):
    obs_pos = pose_convert('grid', 'arena', waypoint)
    obs = Box(
        position=(obs_pos[0], obs_pos[1]+0.15, obs_pos[2]),
        scale=(0.25, 0.25, 0.25),
        color=(50, 60, 200),
        material=Material(opacity=0.3, transparent=True)
    )
    scene.add_object(obs)


def user_join_callback(scene, cam, msg):
    global HUD_distance, distance_to_obstacle
    if "camera" in cam.object_id:
        # circle1 = Circle(
        #     parent=cam.object_id,
        #     position=(0, -0.1, -0.5),
        #     scale=(0.02, 0.02, 0.02)
        # )

        # scene.add_object(circle1)
        HUD_distance = Text(
            parent=cam.object_id,
            object_id="distance",
            text = str(round(distance_to_obstacle, 1)) +' cm',
            position=(0, -0.4, -1.5)
        )
        scene.add_object(HUD_distance)


@ scene.run_async
# @scene.run_once
async def main():
    global loop, rvr, grid
    loop = asyncio.get_event_loop()
    rvr = SpheroRvrAsync(
        dal=SerialAsyncDal(
            loop
        )
    )
    grid = GridMap(10, 10)  # initialize grid map
    init()
    scene.user_join_callback = user_join_callback
    await rvr_setup()


# start tasks
scene.run_tasks()
