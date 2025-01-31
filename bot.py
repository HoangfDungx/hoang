from typing import Tuple

from pygame import Vector2, Surface
import pygame

from ...bot import Bot
from ...linear_math import Transform

import numpy as np
import math
import time

MAX_LOOKAHEAD = 5
MAX_LOOKAHEAD_DIST = 500

MAX_VEL = 2000
VEL_INCREMENT = 1000

class VietRacer(Bot):
    @property
    def name(self):
        return "VietRacer"

    @property
    def contributor(self):
        return "Hoang"
    
    def __init__(self, track):
        super().__init__(track)

        self.vel = 0
        self.target_point = Vector2([0, 0])
        self.current_point = Vector2([0, 0])

    def compute_commands(self, next_waypoint: int, position: Transform, velocity: Vector2) -> Tuple:
        min_vel = self.track.track_width
        r_vel = position.inverse().M * velocity
        
        target = self.track.lines[next_waypoint]
        next_targets = []
        next_targets.append(position.p)
        next_targets.append(target)
        track_length = len(self.track.lines)
        
        # Find next targets
        for id in range(next_waypoint + 1, next_waypoint + MAX_LOOKAHEAD + 1):
            next_target = self.track.lines[id % track_length]
            dist = next_target.distance_to(target)
            if (next_target.distance_to(target) > MAX_LOOKAHEAD_DIST):
                break

            next_targets.append(next_target)

        limit1 = self.vel + VEL_INCREMENT
        limit2 = MAX_VEL
        print(f"limit: {limit1}")
        print(f"len(next_targets) {len(next_targets)}")
        target_sgn = 1

        # Steering angle
        steering_angle = 0
        if (len(next_targets) > 2):
            target_vect = next_targets[-1] - target
            target_vect = target_vect.normalize() * self.track.track_width * 0.6             # target point must be inside track radius
            target_point_1 = target + target_sgn * target_vect
            target_vect = next_targets[-1] - next_targets[2]
            if target_vect.length() > 1e-6:
                target_vect = target_vect.normalize() * self.track.track_width * 0.6             # target point must be inside track radius
            target_point_2 = next_targets[2] + target_sgn * target_vect
            # Check target point 2 is possible
            if self.point_to_line_distance(position.p, target_point_2, target) < 0.6 * self.track.track_width:
                self.target_point = target_point_2
            else:
                self.target_point = target_point_1
            self.current_point = position.p
            car_target = position.inverse() * self.target_point
            steering_angle = car_target.as_polar()[1]
            print(steering_angle * np.pi / 180)
        else:
            car_target = position.inverse() * target
            steering_angle = car_target.as_polar()[1]

        # Different angle between targets
        if (len(next_targets) > 2):
            for i in range(1, len(next_targets) - 2):
                print(i)
                turing_radius = self.circumradius(
                    next_targets[1], next_targets[i + 1], next_targets[i + 2]
                )
                print(f"turing_radius: {turing_radius}")
                limit2 = min(limit2, 1.1 ** (i) * turing_radius / 1.5)
                print(f"Limit {i}: {limit2}")

                turing_radius = self.circumradius(
                    next_targets[i], next_targets[i + 1], next_targets[i + 2]
                )
                print(f"turing_radius: {turing_radius}")
                limit2 = min(limit2, 1.1 ** (i) * turing_radius / 1.5)
                print(f"Limit {i}: {limit2}")

                v1 = next_targets[1] - next_targets[0]
                v2 = next_targets[i + 2] - next_targets[0]
                angle = v1.angle_to(v2)
                limit2 = min(limit2, self._limit_vel(angle))

        limit2 = limit2 + 12 * self.target_point.distance_to(position.p) ** 0.55
        limit = min(limit1, limit2)

        steering_angle = steering_angle * (1 + abs(r_vel.y) / 40)
        # print(f"vel: {self.vel}, steering angle: {steering_angle}")
        limit = min(limit, MAX_VEL * np.cos(steering_angle * np.pi / 180))
        self.vel = limit
        print(f"Final vel: {self.vel}")
        print(f"Actual vel: {r_vel}")
        throttle = (self.vel - abs(r_vel.x)) * 1
        throttle = np.clip(throttle, -1, 1)
        
        # calculate the steering
        return throttle, steering_angle
        if steering_angle > 0:
            return throttle, 1
        else:
            return throttle, -1
    
    @staticmethod
    def _limit_vel(angle, min_vel = 0):
        angle = angle * np.pi / 180
        return np.cos(angle) ** 2 * MAX_VEL + 4 * min_vel
    
    @staticmethod
    def circumradius(A, B, C):
        # Calculate side lengths
        AB = A.distance_to(B)
        BC = B.distance_to(C)
        CA = C.distance_to(A)
        
        # Semi-perimeter
        s = (AB + BC + CA) / 2
        
        # Area using Heron's formula
        area = math.sqrt(s * (s - AB) * (s - BC) * (s - CA))
        
        # Circumradius formula
        if area == 0:  # Degenerate triangle
            return 1e6
        R = (AB * BC * CA) / (4 * area)
        return R
    
    @staticmethod
    def point_to_line_distance(A, B, P):
        # Line vector (B - A)
        line_vec = B - A
        
        # Point vector (P - A)
        point_vec = P - A
        
        # Line length
        line_length = line_vec.length()
        
        if line_length == 0:  # A and B are the same point
            return P.distance_to(A)
        
        # Area of parallelogram (cross product of 2D vectors)
        area = abs(line_vec.x * point_vec.y - line_vec.y * point_vec.x)
        
        # Distance = height of parallelogram / base length
        distance = area / line_length
        return distance
    
    def draw(self, map_scaled: Surface, zoom):
        pygame.draw.lines(map_scaled, (100, 0, 0), False, [
            zoom * self.current_point, zoom * self.target_point], 2)