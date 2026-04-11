"""Loads lane_params.yaml and provides config access."""

import yaml
from dataclasses import dataclass
from typing import Tuple

@dataclass
class LaneConfig:
    # Image
    img_width: int
    img_height: int
    strip_top_fraction: float
    
    # Segmentation
    hsv_low: Tuple[int, int, int]
    hsv_high: Tuple[int, int, int]
    morph_close_iterations: int
    
    # Detection
    corridor_px: int
    split_fraction: float
    lane_width_px: int
    
    # Control
    kp: float
    kd: float
    drive_speed: float
    max_error_to_drive: float
    no_detection_stop_frames: int
    
    # Topics
    topic_image: str
    topic_cmd_vel: str
    
    # Debug
    show_debug: bool


def load_config(yaml_path: str) -> LaneConfig:
    with open(yaml_path, 'r') as f:
        cfg = yaml.safe_load(f)
    
    return LaneConfig(
        img_width=cfg['image']['width'],
        img_height=cfg['image']['height'],
        strip_top_fraction=cfg['image']['strip_top_fraction'],
        hsv_low=tuple(cfg['segmentation']['hsv_low']),
        hsv_high=tuple(cfg['segmentation']['hsv_high']),
        morph_close_iterations=cfg['segmentation']['morph_close_iterations'],
        corridor_px=cfg['detection']['corridor_px'],
        split_fraction=cfg['detection']['split_fraction'],
        lane_width_px=cfg['detection']['lane_width_px'],
        kp=cfg['control']['kp'],
        kd=cfg['control']['kd'],
        drive_speed=cfg['control']['drive_speed'],
        max_error_to_drive=cfg['control']['max_error_to_drive'],
        no_detection_stop_frames=cfg['control']['no_detection_stop_frames'],
        topic_image=cfg['topics']['image'],
        topic_cmd_vel=cfg['topics']['cmd_vel'],
        show_debug=cfg['debug']['show_display'],
    )
