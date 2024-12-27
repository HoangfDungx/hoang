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

MAX_VEL = 600
MIN_VEL = 100
VEL_INCREMENT = 10

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
        # print(f"limit: {limit1}")

        # Different angle between targets
        for i in range(len(next_targets) - 2):
            v1 = next_targets[i + 1] - next_targets[i]
            v2 = next_targets[i + 2] - next_targets[i + 1]
            angle = v1.angle_to(v2)
            limit2 = min(limit2, self._limit_vel(angle))
            # print(f"angle: {angle}, limit {i}: {limit2}")
            v1 = next_targets[i + 1] - next_targets[i]
            v2 = next_targets[i + 2] - next_targets[i]
            angle = v1.angle_to(v2)
            limit2 = min(limit2, self._limit_vel(angle))
            # print(f"angle: {angle}, limit {i}: {limit2}")
            v1 = next_targets[1] - next_targets[0]
            v2 = next_targets[i + 2] - next_targets[0]
            angle = v1.angle_to(v2)
            limit2 = min(limit2, self._limit_vel(angle))
            # print(f"angle: {angle}, limit {i}: {limit2}")

        limit2 = limit2 + 0.8 * target.distance_to(position.p)
        limit = min(limit1, limit2)

        # Steering angle
        steering_angle = 0
        if (len(next_targets) > 2):
            target_vect = next_targets[2] - target
            target_vect = target_vect.normalize() * self.track.track_width * 0.8             # target point must be inside track radius
            self.target_point = target + target_vect
            self.current_point = position.p
            car_target = position.inverse() * self.target_point
            steering_angle = car_target.as_polar()[1]
            # print(steering_angle * np.pi / 180)
        else:
            car_target = position.inverse() * target
            steering_angle = car_target.as_polar()[1]
        # print(f"vel: {self.vel}, steering angle: {steering_angle}")

        limit = min(limit, self._limit_vel(steering_angle) - MIN_VEL)
        self.vel = limit
        throttle = (self.vel - velocity.length()) * 0.1
        throttle = np.clip(throttle, -1, 1)
        
        # calculate the steering
        return throttle, steering_angle
        if steering_angle > 0:
            return throttle, 1
        else:
            return throttle, -1
    
    @staticmethod
    def _limit_vel(angle):
        angle = angle * np.pi / 180
        return ((1 - np.tanh(abs(angle) ** 3)) ** 2) * MAX_VEL + MIN_VEL
    
    def draw(self, map_scaled: Surface, zoom):
        pygame.draw.lines(map_scaled, (100, 0, 0), False, [
            zoom * self.current_point, zoom * self.target_point], 2)