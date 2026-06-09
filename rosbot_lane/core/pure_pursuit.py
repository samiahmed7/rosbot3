"""Pure Pursuit controller with rotate-first hybrid for forward, direct for reverse."""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class PurePursuitConfig:
    """Configuration for Pure Pursuit controller."""
    lookahead_distance: float = 0.5
    min_lookahead: float = 0.3
    max_lookahead: float = 1.0
    
    max_speed: float = 0.15
    min_speed: float = 0.03
    curve_speed: float = 0.08
    reverse_speed: float = 0.10  # Slower for reverse
    
    max_angular: float = 1.0
    kp_angular: float = 1.5
    
    # Rotate first threshold (forward only)
    rotate_first_threshold: float = 0.3  # ~17 degrees
    
    goal_tolerance: float = 0.12
    angle_tolerance: float = 0.10


class PurePursuitController:
    """
    Pure Pursuit controller.
    - Forward: rotate-first if heading error > threshold
    - Reverse: always drive (no rotate-first)
    """
    
    def __init__(self, config: PurePursuitConfig = None):
        self.cfg = config or PurePursuitConfig()
    
    def compute_control(
        self,
        current_x: float,
        current_y: float,
        current_theta: float,
        target_x: float,
        target_y: float,
        is_reverse: bool = False,
        speed_factor: float = 1.0
    ) -> Tuple[float, float, float, float, bool]:
        """
        Compute control commands.
        
        Args:
            current_x, current_y: Robot position
            current_theta: Robot heading
            target_x, target_y: Target waypoint
            is_reverse: True if robot should drive backwards
            speed_factor: Speed multiplier (0-1)
        
        Returns:
            (linear_vel, angular_vel, distance, heading_error, reached)
        """
        # Distance to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx * dx + dy * dy)
        
        # Check if reached
        if distance < self.cfg.goal_tolerance:
            return 0.0, 0.0, distance, 0.0, True
        
        # Angle to target (from robot to target)
        angle_to_target = math.atan2(dy, dx)
        
        if is_reverse:
            # REVERSE: Robot faces one way but moves opposite
            # We want the BACK of the robot to point toward target
            # So heading error is: (angle_to_target + π) - current_theta
            desired_heading = self._normalize_angle(angle_to_target + math.pi)
            heading_error = self._normalize_angle(desired_heading - current_theta)
            
            # For reverse, always drive (no rotate-first)
            # Use smaller lookahead for tighter control
            lookahead = min(self.cfg.min_lookahead, distance)
            
            # Curvature for reverse
            if lookahead > 0.01:
                # Note: sign is flipped for reverse steering
                curvature = 2.0 * math.sin(heading_error) / lookahead
            else:
                curvature = 0.0
            
            # Speed (negative for reverse)
            base_speed = self.cfg.reverse_speed * speed_factor
            linear = -base_speed  # Negative = reverse
            
            # Angular (same sign convention, steering feels natural)
            angular = curvature * base_speed  # Use positive speed for calculation
            angular = max(-self.cfg.max_angular, min(self.cfg.max_angular, angular))
            
        else:
            # FORWARD: Normal Pure Pursuit with rotate-first
            heading_error = self._normalize_angle(angle_to_target - current_theta)
            
            # Rotate first if large error
            if abs(heading_error) > self.cfg.rotate_first_threshold:
                linear = 0.0
                angular = self.cfg.kp_angular * heading_error
                angular = max(-self.cfg.max_angular, min(self.cfg.max_angular, angular))
            else:
                # Pure Pursuit
                lookahead = min(self.cfg.lookahead_distance, distance)
                lookahead = max(self.cfg.min_lookahead, lookahead)
                
                if lookahead > 0.01:
                    curvature = 2.0 * math.sin(heading_error) / lookahead
                else:
                    curvature = 0.0
                
                linear = self.cfg.max_speed * speed_factor
                linear = max(self.cfg.min_speed, min(linear, self.cfg.max_speed))
                
                angular = curvature * linear
                angular = max(-self.cfg.max_angular, min(self.cfg.max_angular, angular))
        
        return linear, angular, distance, heading_error, False
    
    def compute_align_control(
        self,
        current_theta: float,
        target_theta: float
    ) -> Tuple[float, bool]:
        """Align heading to target."""
        heading_error = self._normalize_angle(target_theta - current_theta)
        
        if abs(heading_error) < self.cfg.angle_tolerance:
            return 0.0, True
        
        angular = self.cfg.kp_angular * heading_error
        angular = max(-self.cfg.max_angular, min(self.cfg.max_angular, angular))
        
        return angular, False
    
    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle