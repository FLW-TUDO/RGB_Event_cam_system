import numpy as np
import open3d as o3d


def create_point_cloud_with_spheres(points, radius=0.01):
    """
    Create an Open3D point cloud with spheres representing each point.

    Args:
        points (np.ndarray): The 3D points to represent.
        radius (float): The radius of the spheres.

    Returns:
        o3d.geometry.TriangleMesh: A mesh with spheres representing the points.
    """
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(points)

    # Create spheres for each point
    spheres = []
    for point in points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius)
        sphere.translate(point)
        spheres.append(sphere)

    # Combine all spheres into one mesh
    combined_mesh = o3d.geometry.TriangleMesh()
    for sphere in spheres:
        combined_mesh += sphere

    return combined_mesh

def visualize_point_clouds(points_1, points_2):
    pcd1_spheres = create_point_cloud_with_spheres(points_1, radius=0.02)  # Larger radius
    pcd2_spheres = create_point_cloud_with_spheres(points_2, radius=0.02)

    # Optionally, set different colors for each point cloud
    pcd1_spheres.paint_uniform_color([1, 0, 0])  # Red
    pcd2_spheres.paint_uniform_color([0, 1, 0])  # Green

    # Create Open3D visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Combined 3D Point Clouds', width=800, height=600)

    # Add both point clouds to the visualizer
    vis.add_geometry(pcd1_spheres)
    vis.add_geometry(pcd2_spheres)

    vis.update_geometry(pcd1_spheres)
    vis.update_geometry(pcd2_spheres)
    vis.poll_events()
    vis.update_renderer()
    # Run the visualizer
    vis.run()