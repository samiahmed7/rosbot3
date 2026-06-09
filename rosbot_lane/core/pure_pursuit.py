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
        
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx * dx + dy * dy)
        
        if distance < self.cfg.goal_tolerance:
            return 0.0, 0.0, distance, 0.0, True
        
        angle_to_target = math.atan2(dy, dx)
        
        # Heading error depends on drive direction
        if is_reverse:
            # Back of robot points at target → desired heading is rotated 180°
            desired_heading = self._normalize_angle(angle_to_target + math.pi)
            heading_error = self._normalize_angle(desired_heading - current_theta)
            base_speed = self.cfg.reverse_speed
        else:
            heading_error = self._normalize_angle(angle_to_target - current_theta)
            base_speed = self.cfg.max_speed
        
        # Pure Pursuit curvature — uses ACTUAL distance to lookahead point
        if distance > 0.01:
            curvature = 2.0 * math.sin(heading_error) / distance
        else:
            curvature = 0.0
        
        # Smooth speed scaling: cos(err) is 1.0 aligned, 0.5 at 60°, 0 at 90°.
        # Floor at 0.3 so it never freezes mid-curve.
        speed_scale = max(0.3, math.cos(heading_error))
        linear_magnitude = base_speed * speed_factor * speed_scale
        linear_magnitude = max(self.cfg.min_speed, min(linear_magnitude, base_speed))
        
        linear = -linear_magnitude if is_reverse else linear_magnitude
        angular = curvature * linear_magnitude
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