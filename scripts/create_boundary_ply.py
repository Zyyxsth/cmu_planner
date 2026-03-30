#!/usr/bin/env python3
"""
Create a boundary PLY file for exploration constraint.

This script generates a PLY file defining the exploration boundary polygon.
The boundary constrains where the robot can explore, preventing it from
exploring outside the indoor area.

Usage:
    python3 create_boundary_ply.py <x1> <y1> <x2> <y2> ... <xn> <yn> -o <output_file>
    python3 create_boundary_ply.py --rectangle <x_min> <y_min> <x_max> <y_max> -o <output_file>
    
Example:
    # Create a rectangular boundary from (-10, -10) to (30, 30)
    python3 create_boundary_ply.py --rectangle -10 -10 30 30 -o my_boundary.ply
    
    # Create a custom polygon boundary
    python3 create_boundary_ply.py -10 -10 30 -10 30 30 -10 30 -o my_boundary.ply
"""

import argparse
import sys


def create_boundary_ply(points, output_file):
    """
    Create a PLY file from boundary points.
    
    Args:
        points: List of (x, y) tuples defining the boundary polygon
        output_file: Path to output PLY file
    """
    # Ensure the polygon is closed (first point == last point)
    if points[0] != points[-1]:
        points = list(points) + [points[0]]
    
    num_vertices = len(points)
    
    with open(output_file, 'w') as f:
        # Write PLY header
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {num_vertices}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        
        # Write vertices
        for x, y in points:
            f.write(f"{x:.1f}\t{y:.1f}\t0\n")
    
    print(f"Boundary file created: {output_file}")
    print(f"Number of vertices: {num_vertices}")
    print(f"Boundary area: {calculate_area(points):.2f} square meters")


def calculate_area(points):
    """Calculate polygon area using shoelace formula."""
    area = 0
    n = len(points)
    for i in range(n - 1):
        area += points[i][0] * points[i+1][1]
        area -= points[i+1][0] * points[i][1]
    return abs(area) / 2


def main():
    parser = argparse.ArgumentParser(
        description='Create a boundary PLY file for exploration constraint.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Rectangular boundary
  %(prog)s --rectangle -10 -10 40 30 -o indoor_boundary.ply
  
  # Custom polygon (must specify even number of coordinates)
  %(prog)s -10 -10 40 -10 40 30 20 30 20 10 -10 10 -o l_shape.ply
  
  # Use with mtare_planner launch file
  ros2 launch vehicle_simulator system_simulation_with_mtare_planner_multi_robot.launch.py \\
      use_boundary:=true \\
      boundary_file:=/path/to/indoor_boundary.ply
        """
    )
    
    parser.add_argument('coords', nargs='*', type=float,
                        help='Polygon coordinates as x1 y1 x2 y2 ... (must be even number)')
    parser.add_argument('-o', '--output', default='boundary.ply',
                        help='Output PLY file path (default: boundary.ply)')
    parser.add_argument('--rectangle', nargs=4, type=float, metavar=('X_MIN', 'Y_MIN', 'X_MAX', 'Y_MAX'),
                        help='Create rectangular boundary (x_min y_min x_max y_max)')
    
    args = parser.parse_args()
    
    if args.rectangle:
        x_min, y_min, x_max, y_max = args.rectangle
        points = [
            (x_min, y_min),
            (x_max, y_min),
            (x_max, y_max),
            (x_min, y_max),
            (x_min, y_min),  # Close the polygon
        ]
    elif args.coords:
        if len(args.coords) % 2 != 0:
            print("Error: Must provide even number of coordinates (x1 y1 x2 y2 ...)")
            sys.exit(1)
        if len(args.coords) < 6:
            print("Error: Polygon must have at least 3 points (6 coordinates)")
            sys.exit(1)
        points = [(args.coords[i], args.coords[i+1]) for i in range(0, len(args.coords), 2)]
        # Auto-close if needed
        if points[0] != points[-1]:
            points.append(points[0])
    else:
        parser.print_help()
        sys.exit(1)
    
    create_boundary_ply(points, args.output)
    print(f"\nTo use this boundary with mtare_planner:")
    print(f"  ros2 launch vehicle_simulator system_simulation_with_mtare_planner_multi_robot.launch.py \\")
    print(f"    use_boundary:=true \\")
    print(f"    boundary_file:={args.output}")


if __name__ == '__main__':
    main()
