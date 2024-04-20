from math import dist
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path

import numpy as np
from pyquaternion import Quaternion

from collections import deque

max_mps = .45
turn_diameter = .21
turn_radius = turn_diameter / 2

def get_quaternion(pose):
    o = pose.pose.orientation
    return Quaternion([o.w, o.x, o.y, o.z])

def get_position_array(pose):
    p = pose.pose.position
    return np.array([p.x, p.y, p.z])

def yaw_from_quat(q):
    return np.arctan2(2.0 * (q.x * q.y + q.w * q.z), q.w**2 + q.x**2 - q.y**2 - q.z**2)

def pose_distance(pose1, pose2):
    p1 = pose1.pose.position
    p2 = pose2.pose.position
    yaw = yaw_from_quat(get_quaternion(pose1))
    target_yaw = np.arctan2(p2.y - p1.y, p2.x - p1.x)
    turn_angle = min((yaw - target_yaw) % np.pi, (target_yaw - yaw) % np.pi)
    linear_dist = np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
    rot_dist = turn_diameter * turn_angle / 2
    return linear_dist + rot_dist

def intersect_lines(line1, line2):
    (x11, y11, _), (x12, y12, _) = line1
    (x21, y21, _), (x22, y22, _) = line2
    a1 = y12 - y11
    b1 = x11 - x12
    c1 = a1 * x11 + b1 * y11

    a2 = y22 - y21
    b2 = x21 - x22
    c2 = a2 * x21 + b2 * y21

    determinant = a1*b2 - a2*b1

    if determinant == 0:
        return None
    else:
        x = (b2*c1 - b1*c2)/determinant
        y = (a1*c2 - a2*c1)/determinant
        return x,y
    
def find_closest_line(lines, point):
    line_idx, start_or_end = point
    pos = lines[line_idx][start_or_end]
    closest_dist = np.inf
    closest_line = None
    closest_line_idx = None
    for i, line in enumerate(lines):
        if i == line_idx:
            continue
        # 1 - start_or_end because starts of lines should match with ends of other lines and vice versa
        dist = np.linalg.norm(pos - line[1 - start_or_end])
        if dist < closest_dist:
            closest_dist = dist
            closest_line = line
            closest_line_idx = i
    return closest_line, closest_line_idx

def get_rotation_matrix(theta):
    return np.array([[np.cos(theta), -np.sin(theta)]
                    ,[np.sin(theta), np.cos(theta)]])

def get_rotation_direction(hull):
    total_turn = 0
    for i, p1 in enumerate(hull):
        p2 = hull[(i+1) % len(hull)]
        p3 = hull[(i+2) % len(hull)]
        theta1 = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
        theta2 = np.arctan2(p3[1] - p2[1], p3[0] - p2[0])
        angle = min((theta1 - theta2) % (2*np.pi), (theta2 - theta1) % (2*np.pi))
        if (theta1 - theta2) % (2*np.pi) < (theta2 - theta1) % (2*np.pi):
            total_turn -= angle
        else:
            total_turn += angle
    return total_turn < 0

class Zone():
    def __init__(self, path):
        self.path = path
        self.starting_point = None
        self.rotation_matrix = None  
        self.clockwise = None

    def stride_path(self):
        # clean up path into hull
        self.remove_implausible_poses()
        perimeter_lines = self.find_perimeter_lines()
        hull = self.create_hull(perimeter_lines)

        #grid off the zone
        grid = self.create_grid(hull)
        breakpoint()

    def remove_implausible_poses(self):
        keep_path = []
        i = 1
        while i < len(self.path):
            pose = self.path[i]
            p = pose.pose.position
            t = pose.header.stamp.sec + 1e-9 * pose.header.stamp.nanosec
            start_idx = max(0, i-50)
            valid = True
            for j in range(start_idx, i):
                prev_pose = self.path[j]
                dist = pose_distance(prev_pose, pose)
                prev_t = prev_pose.header.stamp.sec + 1e-9 * prev_pose.header.stamp.nanosec
                t_delta = t - prev_t
                if dist / t_delta > max_mps:
                    valid = False
                    break
            if valid:
                keep_path.append(pose)
            i += 1
        self.path = keep_path

    def find_perimeter_lines(self):
        # a redundant pose is one that is along a line since navigating from the start
        # of that line to the end of that line would necessitate going through that pose
        segments = deque([self.path])
        lines = []
        while len(segments):
            poses = segments.popleft()
            longest_line, longest_line_indices, longest_line_dist = self.find_longest_line(poses)
            if longest_line is None or longest_line_dist < 0.1:
                continue
            lines.append(longest_line)
            start_idx, end_idx = longest_line_indices
            # if there are still poses at the beginning of the list, then add them as a new segment
            if len(poses[:start_idx]) > 1:
                segments.append(poses[:start_idx])
            # if there are still poses at the end of the list, then add them as a new segment
            if end_idx + 2 < len(poses):
                segments.append(poses[end_idx+1:])
        return lines

    def find_longest_line(self, poses=None):
        if poses is None:
            poses = self.path
        longest_line = None
        longest_line_indices = None
        longest_line_dist = 0
        quats = [get_quaternion(p) for p in poses]
        for i, q in enumerate(quats):
            dist = 0
            p = poses[i]
            for j in range(i+1, len(quats)):
                q_next = quats[j]
                if Quaternion.distance(q, q_next) < np.pi/60:
                    pos_start = get_position_array(p)
                    pos_end = get_position_array(poses[j])
                    dist = max(dist, np.linalg.norm(pos_end - pos_start))
                    if dist > longest_line_dist:
                        longest_line = (pos_start, pos_end)
                        longest_line_indices = (i, j)
                        longest_line_dist = dist
        return longest_line, longest_line_indices, longest_line_dist
    
    def create_hull(self, lines, last_endpoint=np.zeros(3)):
        # start with the point closest to the origin (or the point closest to the end of the last zone)
        # Move in the same direction as the lines finding the corners of the hull that encloses the zone
        closest_point = None
        closest_dist = np.inf
        for i, line in enumerate(lines):
            start_pos, end_pos = line
            start_dist = np.linalg.norm(start_pos - last_endpoint)
            if start_dist < closest_dist:
                closest_dist = start_dist
                closest_point = (i, 0)
            end_dist = np.linalg.norm(end_pos - last_endpoint)
            if end_dist < closest_dist:
                closest_dist = end_dist
                closest_point = (i, 1)

        hull = []
        while len(hull) < len(lines):
            closest_line, closest_line_idx = find_closest_line(lines, closest_point)
            line_idx, start_or_end = closest_point
            p = intersect_lines(lines[line_idx], closest_line)
            hull.append(p)
            if self.starting_point is None:
                self.starting_point = p
            if start_or_end == 0:
                closest_point = (line_idx, 1)
            else:
                closest_point = (closest_line_idx, 1)

        return hull
    
    def create_grid(self, hull):
        #first, find axes with which to transform the grid. This is based on the longest line in the hull
        longest_line = None
        longest_dist = 0
        for i, p in enumerate(hull):
            next_p = hull[(i+1)%len(hull)]
            dist = np.sqrt((p[0] - next_p[0]) ** 2 + (p[1] - next_p[1])**2)
            if dist > longest_dist:
                longest_dist = dist
                longest_line = (p, next_p)
        p1, p2 = longest_line
        idx = np.argmin([np.linalg.norm(p1), np.linalg.norm(p2)])
        theta = np.arctan2(longest_line[1-idx][1] - longest_line[idx][1]
                           , longest_line[1-idx][0] - longest_line[idx][0])
        self.rotation_matrix = get_rotation_matrix(theta)
        #Translate all points into the rotated coordinate frame
        hull = [np.matmul(self.rotation_matrix, np.array(p)- np.array(self.starting_point)) for p in hull]
        min_x = np.min([p[0] for p in hull])
        min_y = np.min([p[1] for p in hull])

        #Make sure all points are positive
        self.offset = np.array([min_x, min_y])
        hull = [p - self.offset for p in hull]
        max_x = np.max([p[0] for p in hull])
        max_y = np.max([p[1] for p in hull])
        self.grid = np.zeros((int(max_x / turn_radius) + 1, int(max_y / turn_radius) + 1))

        # mark the perimeter with -1
        for i, p in enumerate(hull):
            next_p = hull[(i+1) % len(hull)]
            dist = np.linalg.norm(next_p - p)
            for t in np.arange(0.0, 1.0, turn_radius/dist):
                x, y = p * t + next_p * (1 - t)
                x, y = int(x/turn_radius), int(y/turn_radius)
                self.grid[x,y] = -1
            x, y = next_p
            x, y = int(x/turn_radius), int(y/ turn_radius)
            self.grid[x,y] = -1

        def check_interior(grid, i, j, add, axis):
            k = 1
            while True:
                idx = i if axis == 0 else j
                idx = idx + (k if add else -k)
                if (add and idx >= grid.shape[axis]) or (not add and idx < 0):
                    return False
                if (axis == 0 and grid[idx, j] in [-1,1]) or (axis == 1 and grid[i, idx] in [-1,1]):
                    return True
                k += 1

        # mark the interior with 1
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                if self.grid[i,j] == -1:
                    continue
                # check -x direction
                if check_interior(self.grid, i, j, False, 0) \
                    and check_interior(self.grid, i, j, True, 0) \
                    and check_interior(self.grid, i, j, False, 1) \
                    and check_interior(self.grid, i, j, True, 1):
                    self.grid[i,j] = 1



    def stride_path(self):
        class Stride():
            def __init__(self, p1, p2):
                self.p1 = p1
                self.p2 = p2

        strides = []
        for i in range(len(self.grid.shape[0])):
            j = 0
            while j < self.grid.shape[1]:
                if self.grid[i,j] == 1:
                    stride = [(i,j)]
                    while self.grid[i,j] == 1:
                        j += 1
                    stride.append((i, j-1))
                    strides.append(stride)
                j += 1
        path = []         

    def save_path(self, file):
        import pickle as pkl
        pkl.dump(self.path, open(file,'wb'))

    def plot_perimeter(self):
        import matplotlib.pyplot as plt
        xs, ys = [], []
        for pose in self.path:
            p = pose.pose.position
            xs.append(p.x)
            ys.append(p.y)
        plt.scatter(xs, ys, color='green')
        plt.show()
            

class MowingPlanner(Node):
    def __init__(self):
        super().__init__('mowing_planner')
        
        self.perimeter_sub_ = self.create_subscription(
            Path,
            '/visual_slam/tracking/perimeter',
            self.process_perimeter,
            10
        )

        self.zones = []

    def process_perimeter(self, path_msg):
        self.zones.append(Zone(path_msg.poses))

def main(args=None):
    rclpy.init(args=args)

    mowing_planner = MowingPlanner()
    
    rclpy.spin(mowing_planner)

    mowing_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    