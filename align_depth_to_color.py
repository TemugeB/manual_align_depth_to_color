import pyrealsense2 as rs
import numpy as np
import cv2

# Create a pipeline
pipeline = rs.pipeline()

#set the camera resolutions
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# Start stream
profile = pipeline.start()

#get the camera intrinsics and extrinsics
depth_profile = pipeline.get_active_profile().get_stream(rs.stream.depth).as_video_stream_profile()
depth_intrinsics = depth_profile.get_intrinsics()

color_profile = pipeline.get_active_profile().get_stream(rs.stream.color).as_video_stream_profile()
color_intrinsics = color_profile.get_intrinsics()

depth_to_color_extrinsics = pipeline.get_active_profile().get_stream(rs.stream.depth).get_extrinsics_to(color_profile)


#precalculate values needed to convert depth image to pointcloud
def prep_XY():

    #focal point and center of frame
    height = depth_intrinsics.height
    width = depth_intrinsics.width
    ppx = depth_intrinsics.ppx
    ppy = depth_intrinsics.ppy
    fx = depth_intrinsics.fx
    fy = depth_intrinsics.fy

    #create grid for point_cloud calculation
    points_xy = np.mgrid[0:height, 0:width].reshape((2, -1)).T

    #make the grid points centered
    points_xy = points_xy - np.array([ppy, ppx])
    points_x = points_xy[:,1]
    points_y = points_xy[:,0]

    #scaled_points
    points_x = points_x/fx
    points_y = points_y/fy

    return points_x, points_y


#this code converts depth image to pointcloud.
def depth_to_pointcloud(depth: np.ndarray, points_x: np.ndarray, points_y:np.ndarray, far_limit = 2000.) -> np.ndarray:

    '''
    far_limit is how far the points can be from the camera. 
    In this case, 2000 [milimeter] means points that are more than 2m are clamped to be 2m.
    If your application needs to see farther than 2 meter, then set this value higher.
    Or set to -1 if you dont want to use far_limit.
    '''

    Z = depth.flatten()
    if far_limit > 0:
        Z[Z > far_limit] = far_limit

    #calculate the x,y coordinates of the point cloud
    X = (points_x * Z)
    Y = (points_y * Z)

    #This is the pointcloud
    pcd = np.stack([X,Y,Z], axis = -1)

    return pcd

def align_manual(depth, points_x, points_y):

    #convert the depth image to pointclound in depth camera coordinates
    pcd = depth_to_pointcloud(depth, points_x, points_y)

    #pointcloud in depth camera coordinates to pointcloud in color camera coordinates
    rot = np.array(depth_to_color_extrinsics.rotation).reshape((3,3))
    trans = np.array(depth_to_color_extrinsics.translation).reshape((1,3))
    #translation vector is in units of [meter] while pointcloud is in [milimeter]. So a convertion is necessary
    trans *= 1000.
    pcd_color = pcd @ rot + trans
    

    #Now the pointcloud exists in the coordinates defined where the color camera is at the origin.
    #Therefore, we simply project the pointcloud to image space using the camera intrinsics

    height = color_intrinsics.height
    width = color_intrinsics.width
    ppx = color_intrinsics.ppx
    ppy = color_intrinsics.ppy
    fx = color_intrinsics.fx
    fy = color_intrinsics.fy

    #split the pointcloud into its coordinates along each axis
    Xc = pcd_color[:,0]
    Yc = pcd_color[:,1]
    Zc = pcd_color[:,2]

    #these are indices of depth points that are valid. 
    nonzeros = Zc != 0

    #project to pixel coordinates. Only pointcloud points that are valid.
    #If you don't know how to project to camera pixels, look here:
    #https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#:~:text=Detailed%20Description
    #more specifically, the projection is done with this equation:
    #https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#:~:text=equivalent%20to%20the%20following
    u = fx * (Xc[nonzeros]/Zc[nonzeros]) + ppx
    v = fy * (Yc[nonzeros]/Zc[nonzeros]) + ppy
    #convert to image like indexing
    u = np.round(u).astype(np.int32)
    v = np.round(v).astype(np.int32)

    #Now we are only interested in points that fall inside the color image frame
    u_inbound = (u >=0) * (u < width)
    v_inbound = (v >=0) * (v < height)
    inbound = u_inbound * v_inbound #these are indices of points that are inside the color camera frame
    
    #shortlist the pointcloud to only non-zero and in frame points
    u = u[inbound]
    v = v[inbound]
    z = Zc[nonzeros][inbound] #these are the depth values for each pixel

    #create a new frame for the aligned depth image
    manual_align = np.zeros((height, width), dtype = np.uint16)
    #apply the pixel values
    manual_align[(v, u)] = z

    return manual_align


def main():

    #depth image to pointcloud conversion factors
    points_x, points_y = prep_XY()

    #this object aligns depth image to color frame using realsense methods
    align_to = rs.stream.color
    align = rs.align(align_to)

    while True:
        #depth and color frame together
        frames = pipeline.wait_for_frames()

        #align the depth frame to color frame using realsense library
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        #convert to numpy images
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())        


        #to manually align the depth image, we need the original depth iamge
        depth_original_image = np.asanyarray(frames.get_depth_frame().get_data())

        #manually align the depth image to color frame
        manually_aligned_depth_image = align_manual(depth_original_image, points_x, points_y)
        #apply small median filter if the lines bother you
        #manually_aligned_depth_image = cv2.medianBlur(manually_aligned_depth_image, 3)

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        realsense_aligned = np.where((depth_image_3d > 1000.) | (depth_image_3d <= 0), grey_color, color_image)

        manually_aligned_depth_3d = np.dstack((manually_aligned_depth_image, manually_aligned_depth_image, manually_aligned_depth_image)) #depth image is 1 channel, color is 3 channels
        manually_aligned = np.where((manually_aligned_depth_3d > 1000.) | (manually_aligned_depth_3d <= 0), grey_color, color_image)

        cv2.imshow('librealsense_aligned', realsense_aligned[:,:,[2,1,0]]) #RGB-> BGR
        cv2.imshow('manually_aligned', manually_aligned[:,:,[2,1,0]])
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

if __name__ == '__main__':
    main()
