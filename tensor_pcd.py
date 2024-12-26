# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------

import pyrealsense2 as rs
import numpy as np
from enum import IntEnum
import matplotlib.pyplot as plt

from datetime import datetime
import open3d as o3d

from os.path import abspath
import sys
sys.path.append(abspath(__file__))

my_device = o3d.core.Device("CUDA:0")
my_dtype = o3d.core.float32

def get_intrinsic_tensor(frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    intrinsic_matrix = np.array([
        [intrinsics.fx, 0, intrinsics.ppx],
        [0, intrinsics.fy, intrinsics.ppy],
        [0, 0, 1]
    ])
    return o3d.core.Tensor(intrinsic_matrix, dtype=my_dtype, device=my_device)

if __name__ == "__main__":

    #o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)    # Create a pipeline
    pipeline = rs.pipeline()

    #Create a config and configure the pipeline to stream || different resolutions of color and depth streams
    config = rs.config()
    # Threshold filtering (meters)
    threshold_filter = rs.threshold_filter()
    min_depth = 0.5
    max_depth = 3.5
    threshold_filter.set_option(rs.option.min_distance, min_depth)
    threshold_filter.set_option(rs.option.max_distance, max_depth)
    
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_scale = depth_sensor.get_depth_scale()

 


    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    Vis_PC = o3d.geometry.PointCloud()
    Tensor_PC = o3d.t.geometry.PointCloud(my_device) 
    
    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]

    # Streaming loop
    frame_count = 0

    
    try:
        while True:

            dt0 = datetime.now()
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            #aligned_depth_frame = threshold_filter.process(aligned_depth_frame)
            color_frame = aligned_frames.get_color_frame()
            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            intrinsic_tensor = get_intrinsic_tensor(color_frame)  # Update for each frame if necessary
            # Convert depth frame to tensor-based image
            depth_np = np.asarray(aligned_depth_frame.get_data(), dtype=np.uint16)
            depth_tensor = o3d.t.geometry.Image(o3d.core.Tensor(depth_np, dtype=o3d.core.uint16, device=my_device))
            # Create point cloud directly from depth image
            Tensor_PC = o3d.t.geometry.PointCloud.create_from_depth_image(depth=depth_tensor,intrinsics=intrinsic_tensor,depth_scale=(1.0 / depth_scale),depth_max=max_depth,stride=1, with_normals=False)
            Tensor_PC = Tensor_PC.voxel_down_sample(voxel_size=0.05) #less is more
            Tensor_PC.transform(flip_transform)
            
            if(Tensor_PC.is_cuda):
                print("Tensor is CUDA")
            if(Tensor_PC.is_cpu):
                print("Tensor is CPU")
          
            #Tensor_PC.estimate_normals(max_nn=30, radius=0.2)
            
            #Plane Detection
            plane_model, inliers = Tensor_PC.segment_plane(distance_threshold=0.025, ransac_n=3, num_iterations=125)
            [a, b, c, d] = [float(value) for value in plane_model.cpu().numpy()]            
            print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
            #Height Filtering 
            max_H = 0.61  # 24 inches in meters
            min_H = 0.035  # 2 inches in meters
            # Compute distances to the plane
            points = Tensor_PC.point.positions  # Tensor of shape (N, 3)
            distances = points[:, 0] * a + points[:, 1] * b + points[:, 2] * c + d
            # Create a single mask for height filtering
            mask = (distances > min_H) & (distances < max_H)
            # Apply the mask in one step
            Tensor_PC = Tensor_PC.select_by_mask(mask)


            #Outlier Removal
            RO_removal, RO_mask = Tensor_PC.remove_radius_outliers(nb_points=2, search_radius = 0.25)
            Tensor_PC = RO_removal
            SO_removal, SO_mask = Tensor_PC.remove_statistical_outliers(nb_neighbors=20, std_ratio=2.0)
            Tensor_PC = SO_removal
    
           
            # Clustering Point Cloud
            labels = Tensor_PC.cluster_dbscan(eps=0.21, min_points=60, print_progress=True)

            if labels.shape[0] != 0:  # Check if there are any labels
                labels_np = labels.cpu().numpy()  # Convert labels to NumPy
                max_label = labels_np.max()
                print(f"Found {max_label + 1} clusters in scene")
                # Generate a unique color for each cluster
                num_clusters = max_label + 1

                # Process each cluster for bounding box and dimensions
                for label in range(num_clusters):
                    cluster_indices = np.where(labels_np == label)[0]
                    cluster_indices_tensor = o3d.core.Tensor(cluster_indices, dtype=o3d.core.int64, device=my_device)
                    cluster_points = Tensor_PC.select_by_index(cluster_indices_tensor)

                    # Create bounding box
                    bbox = cluster_points.get_axis_aligned_bounding_box()

                    # Get the max and min bound tensors
                    max_bound = bbox.max_bound
                    min_bound = bbox.min_bound

                    # Convert them to Python scalars using .item()
                    l = max_bound[0].item() - min_bound[0].item()
                    w = max_bound[1].item() - min_bound[1].item()
                    h = max_bound[2].item() - min_bound[2].item()

                    print(f"Object {label + 1} Size l:{l:.2f} w:{w:.2f} h:{h:.2f}")
    
            Leg_PC = Tensor_PC.to_legacy()
            Vis_PC.points = Leg_PC.points
            if frame_count == 0:
                vis.add_geometry(Vis_PC)
                vis.update_geometry(Vis_PC)
                    
                
                
          
                
           
            vis.poll_events()
            vis.update_renderer()

            process_time = datetime.now() - dt0
            print("\rFPS: " + str(1 / process_time.total_seconds()), end='\n')
            frame_count += 1

    finally:
        pipeline.stop()
        vis.destroy_window()