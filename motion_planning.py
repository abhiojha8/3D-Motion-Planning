import argparse
import time
from enum import Enum, auto

import msgpack
import numpy as np
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.frame_utils import global_to_local, local_to_global
from udacidrone.messaging import MsgID

from project_utils import create_grid_2_5d, a_star_2_5d, prune_path, heuristic, \
    visualize_grid_and_pickup_goal, points_collinear_3d, simplify_path, \
    path_2_5d_to_3d_path

TARGET_ALTITUDE = 5
SAFETY_DISTANCE = 5


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        self.interactive_goal = (-122.40017151, 37.7962347, 0)
        self.temporary_scatter = None
        self.previous_location = None
        self.map_grid = None
        self.north_offset = None
        self.east_offset = None
        self.path = None

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 3.0 \
                    and abs(self.target_position[2] - (-self.local_position[2])) < 2.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        print(self.waypoints)
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2],
                          self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def pick_goal(self, event):
        evt = event.mouseevent
        east = int(evt.xdata)
        north = int(evt.ydata)
        alt = self.map_grid[north, east]
        self.interactive_goal = local_to_global(self.grid_coord_to_local_position((north, east, alt)), self.global_home)

        if self.temporary_scatter is not None:
            self.temporary_scatter.remove()
        fig = event.artist.figure
        self.temporary_scatter = fig.gca().scatter(east, north, marker='o', c='g')
        fig.canvas.draw()
        print("You've pick up (lat, lon, alt) {} as the goal. "
              "Close the figure to continue.".format(self.interactive_goal))

    def grid_coord_to_local_position(self, grid_coord):
        lat = grid_coord[0] + self.north_offset
        lon = grid_coord[1] + self.east_offset
        return lat, lon, -grid_coord[2]

    def local_position_to_grid_coord(self, position):
        north = int(position[0] - self.north_offset)
        east = int(position[1] - self.east_offset)
        alt = int(-position[2])
        return north, east, alt

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")

        self.target_position[2] = TARGET_ALTITUDE

        with open('colliders.csv', 'r') as f:
            header_line = f.readline()
            lat_str, lon_str = header_line.split(',')
            lat = float(lat_str.strip().split(' ')[1])
            lon = float(lon_str.strip().split(' ')[1])
            print("Map home location: ({}, {})".format(lat, lon))

        home_position = (lon, lat, 0)
        self.set_home_position(*home_position)

        global_position = self.global_position

        local_position = global_to_local(global_position, home_position)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid_2_5d(data, SAFETY_DISTANCE)
        self.map_grid = grid
        self.north_offset = north_offset
        self.east_offset = east_offset

        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # starting point on the grid
        grid_start = self.local_position_to_grid_coord(local_position)
        alt_start = int(max(TARGET_ALTITUDE, grid_start[2] + 1, grid[grid_start[0], grid_start[1]] + 1))
        grid_start = grid_start[0], grid_start[1], alt_start

        # visualize grid
        # goal will be picked up interactively. But if the user (or, you the reviewer lol)
        # just simply close the grid map, then I've also set a default goal I chose beforehand.
        #
        # comment it out and change self.interactive_goal to what you need if you don't like it
        self.temporary_scatter = visualize_grid_and_pickup_goal(grid, grid_start, self.pick_goal)

        goal = self.interactive_goal
        if len(goal) < 3:
            goal = (goal[0], goal[1], 0)
        goal_local = global_to_local(goal, self.global_home)
        goal_grid = self.local_position_to_grid_coord(goal_local)
        goal_north, goal_east, goal_alt = goal_grid
        grid_goal = (goal_north,
                     goal_east,
                     int(max(grid[goal_north, goal_east] + 1, TARGET_ALTITUDE, goal_alt + 1)))

        print('Start and goal in the grid', grid_start, grid_goal)
        print("Searching path ... Please be patient")
        t0 = time.time()
        path = a_star_2_5d(grid, heuristic, grid_start, grid_goal, TARGET_ALTITUDE)
        print("Path planned by 2.5D A* planner:", path)
        path = path_2_5d_to_3d_path(path)
        print("Path in 3D:", path)
        path = prune_path(path, points_collinear_3d)
        print("Path after prunning:", path)
        path = simplify_path(grid, path)
        print("Search done. Take {} seconds in total".format(time.time() - t0))
        print(path)
        self.path = path
        waypoints = self.path_to_waypoints(path)
        self.waypoints = waypoints
        self.send_waypoints()

    def path_to_waypoints(self, path):
        # Convert path to waypoints
        waypoints = []
        for i in range(len(path)):
            p = path[i]
            p_prev = path[i - 1] if i > 0 else None
            orientation = 0
            if p_prev is not None:
                orientation = np.arctan2(p[1] - p_prev[1], p[0] - p_prev[0])
            waypoints.append([p[0] + self.north_offset, p[1] + self.east_offset, p[2], orientation])
        return waypoints

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
