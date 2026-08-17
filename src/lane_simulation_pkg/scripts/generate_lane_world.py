#!/usr/bin/env python3
"""Generate the segmented circular lane used by the Gazebo simulation."""

import math
from pathlib import Path


TRACK_RADIUS = 20.0
TRACK_CENTER_Y = TRACK_RADIUS
ROAD_WIDTH = 8.0
LANE_HALF_WIDTH = 2.0
SEGMENT_COUNT = 96


def segment_pose(radius, angle):
    """Return the centre and tangent heading of one circular segment."""
    x = radius * math.cos(angle)
    y = TRACK_CENTER_Y + radius * math.sin(angle)
    yaw = angle + math.pi / 2.0
    return x, y, yaw


def box_length(radius, overlap):
    """Return a chord length with a small overlap to hide segment seams."""
    return 2.0 * radius * math.sin(math.pi / SEGMENT_COUNT) + overlap


def road_segment(index, angle):
    """Build one road visual and collision segment."""
    x, y, yaw = segment_pose(TRACK_RADIUS, angle)
    length = box_length(TRACK_RADIUS, 0.08)
    return f"""
        <collision name="road_collision_{index}">
          <pose>{x:.6f} {y:.6f} 0.01 0 0 {yaw:.6f}</pose>
          <geometry><box><size>{length:.6f} {ROAD_WIDTH:.2f} 0.02</size></box></geometry>
        </collision>
        <visual name="road_visual_{index}">
          <pose>{x:.6f} {y:.6f} 0.01 0 0 {yaw:.6f}</pose>
          <geometry><box><size>{length:.6f} {ROAD_WIDTH:.2f} 0.02</size></box></geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.15 0.15 0.15 1</diffuse>
          </material>
        </visual>"""


def lane_segment(name, index, radius, angle):
    """Build one white lane-boundary segment."""
    x, y, yaw = segment_pose(radius, angle)
    length = box_length(radius, 0.06)
    return f"""
        <visual name="{name}_{index}">
          <pose>{x:.6f} {y:.6f} 0.025 0 0 {yaw:.6f}</pose>
          <geometry><box><size>{length:.6f} 0.12 0.01</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
          </material>
        </visual>"""


def generate_world():
    """Return a complete SDF world containing a circular closed track."""
    angles = [
        -math.pi / 2.0 + index * 2.0 * math.pi / SEGMENT_COUNT
        for index in range(SEGMENT_COUNT)
    ]
    road = ''.join(
        road_segment(index, angle) for index, angle in enumerate(angles)
    )
    inner_lane = ''.join(
        lane_segment(
            'inner_lane_line', index, TRACK_RADIUS - LANE_HALF_WIDTH, angle
        )
        for index, angle in enumerate(angles)
    )
    outer_lane = ''.join(
        lane_segment(
            'outer_lane_line', index, TRACK_RADIUS + LANE_HALF_WIDTH, angle
        )
        for index, angle in enumerate(angles)
    )

    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="lane_world">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!--
      Closed circular track: centre=(0, {TRACK_CENTER_Y:.0f}),
      centreline radius={TRACK_RADIUS:.0f} m, lane width={2 * LANE_HALF_WIDTH:.0f} m.
      The robot starts at (0, 0) facing +X, tangent to the centreline.
      Regenerate this file with scripts/generate_lane_world.py.
    -->
    <model name="closed_loop_road">
      <static>true</static>
      <link name="road_link">{road}
      </link>
    </model>

    <model name="lane_markings">
      <static>true</static>
      <link name="lane_markings_link">{inner_lane}{outer_lane}
      </link>
    </model>
  </world>
</sdf>
"""


def main():
    output_path = Path(__file__).parents[1] / 'worlds' / 'lane_world.world'
    output_path.write_text(generate_world(), encoding='utf-8')
    print(f'Generated {output_path} with {SEGMENT_COUNT} track segments.')


if __name__ == '__main__':
    main()
