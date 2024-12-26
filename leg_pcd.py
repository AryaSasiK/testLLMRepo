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

def get_intrinsic_matrix(frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    out = o3d.camera.PinholeCameraIntrinsic(640, 480, intrinsics.fx,
                                            intrinsics.fy, intrinsics.ppx,
                                            intrinsics.ppy)
    return out

if __name__ == "__main__":

    # Create a pipeline
    pipeline = rs.pipeline()

    #Create a config and configure the pipeline to stream || different resolutions of color and depth streams
    config = rs.config()
    # Threshold filtering (meters)
    threshold_filter = rs.threshold_filter()
    min_depth = 0.5
    max_depth = 3
    threshold_filter.set_option(rs.option.min_distance, min_depth)
    threshold_filter.set_option(rs.option.max_distance, max_depth)
    
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()


    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_scale = depth_sensor.get_depth_scale()

    # We will not display the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 3  # 3 meter
    clipping_distance = max_depth / depth_scale

    print(f"\nDepth Scale: {depth_scale }")

    

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    vis = o3d.visualization.Visualizer()
    vis.create_window()


    
    Vis_PC = o3d.geometry.PointCloud() 
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
            aligned_depth_frame = threshold_filter.process(aligned_depth_frame)
            color_frame = aligned_frames.get_color_frame()
            intrinsic = o3d.camera.PinholeCameraIntrinsic(get_intrinsic_matrix(color_frame))
            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = o3d.geometry.Image(np.array(aligned_depth_frame.get_data()))
            all_points = o3d.geometry.PointCloud.create_from_depth_image(depth = depth_image, intrinsic = intrinsic, depth_scale = 1.0 / depth_scale,depth_trunc = max_depth)
            
            #Point Cloud Downsampling 
            all_points = all_points.voxel_down_sample(voxel_size=0.045)
            all_points.transform(flip_transform)
            all_points.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
            
            #Plane Detection
            plane_model, inliers = all_points.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=200)
            [a, b, c, d] = plane_model
            print(f"\nPlane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
           
    
           
           
            #Filtering Point Cloud 
            #Filter points based on hieght from plane
            high_points = np.asarray(all_points.points)
            mask_above = (a * high_points[:, 0] + b * high_points[:, 1] + c * high_points[:, 2] + d) > .61 #Filter Points above 24in = .61m
            all_points = all_points.select_by_index(np.where(mask_above)[0],invert=True)
            
            low_points = np.asarray(all_points.points)
            mask_below = (a * low_points[:, 0] + b * low_points[:, 1] + c * low_points[:, 2] + d) < .05 #Filter Points below 2in = .05m
            all_points = all_points.select_by_index(np.where(mask_below)[0],invert=True)
    
            
            
            #Statistical Outlier Removal 
            S_cl, S_ind = all_points.remove_statistical_outlier(nb_neighbors=20,std_ratio=1.0)
            S_inliers = all_points.select_by_index(S_ind)
            S_outliers = all_points.select_by_index(S_ind, invert=True)
            all_points.points = S_inliers.points
            
            
            #Radius Outlier Removal
            R_cl, R_ind = all_points.remove_radius_outlier(nb_points=2, radius=0.25)
            R_inliers = all_points.select_by_index(R_ind)
            R_outliers = all_points.select_by_index(R_ind, invert=True)
            all_points.points = R_inliers.points
            
    
            
            #Clustering Point Cloud
            labels = np.array(all_points.cluster_dbscan(eps=0.21, min_points=60, print_progress=False))
            if labels.size != 0:
                max_label = labels.max()
                print(f"Found {max_label + 1} clusters in scene")
                colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
                colors[labels < 0] = 0
                all_points.colors = o3d.utility.Vector3dVector(colors[:, :3])
            
                for label in np.unique(labels):
                    if label == -1:  # Skip noise points
                        continue
                    cluster_indices = np.where(labels == label)[0]
                    cluster_points = all_points.select_by_index(cluster_indices)

                    # Create bounding box
                    bbox = cluster_points.get_axis_aligned_bounding_box()
                    [l, w, h] = bbox.get_max_bound() - bbox.get_min_bound()
                    [x,y,z] = bbox.get_center()
                    print(f"Object {label+1} Position: x:{x:.2f} y:{b:.2f} z:{c:.2f}")
                    print(f"Object {label+1} Size l:{l:.2f} w:{w:.2f} h:{h:.2f}")
                    print("\n")
                  
            
            if frame_count == 0:
                vis.add_geometry(all_points)
                vis.update_geometry(all_points)
            
            vis.poll_events()
            vis.update_renderer()

            process_time = datetime.now() - dt0
            print("\rFPS: " + str(1 / process_time.total_seconds()), end='')
            frame_count += 1

    finally:
        pipeline.stop()
        vis.destroy_window()