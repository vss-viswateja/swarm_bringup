#!/usr/bin/env python3
"""
Helper script to convert existing world file aruco boxes from dynamic to static.

This script reads an SDF world file and adds <static>true</static> to all
aruco box includes, optimizing simulation performance.

Usage:
    python3 make_boxes_static.py input_world.sdf output_world.sdf
    
    # Or to modify in-place:
    python3 make_boxes_static.py world.sdf world.sdf
"""

import sys
import re


def make_boxes_static(input_file: str, output_file: str):
    """
    Process an SDF file and add <static>true</static> to all aruco box includes.
    """
    with open(input_file, 'r') as f:
        content = f.read()

    # Pattern to find aruco box includes that don't already have <static>
    # This matches <include> blocks containing "aruco" in the URI
    pattern = r'(<include>\s*<uri>[^<]*aruco[^<]*</uri>\s*<name>([^<]+)</name>\s*)(<pose>)'
    
    def add_static(match):
        before = match.group(1)
        name = match.group(2)
        pose_tag = match.group(3)
        # Add static tag before pose
        return f'{before}<static>true</static>\n      {pose_tag}'
    
    # Apply the replacement
    new_content = re.sub(pattern, add_static, content, flags=re.IGNORECASE)
    
    # Count how many boxes were modified
    original_count = len(re.findall(r'<uri>[^<]*aruco[^<]*</uri>', content, re.IGNORECASE))
    
    with open(output_file, 'w') as f:
        f.write(new_content)
    
    print(f"Processed {original_count} aruco boxes")
    print(f"Output written to: {output_file}")
    print("\nNote: All aruco boxes are now static. Use the object_state_manager")
    print("service to make individual boxes dynamic when needed for manipulation.")


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python3 make_boxes_static.py <input.sdf> <output.sdf>")
        sys.exit(1)
    
    make_boxes_static(sys.argv[1], sys.argv[2])
