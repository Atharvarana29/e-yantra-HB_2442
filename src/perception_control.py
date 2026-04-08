#!/usr/bin/env python3
'''
* Team Id: HB_2442
* Author List: Kumar Sushant Raj, Atharva Rana , Aditya Kumar
* Filename: holonomic_perception.py
* Theme: eYRC Holo Battalion
* Functions: __init__,camera_image_callback(),pixel_to_world(),calculate_marker_orientation(),apply_parallax_correction(),image_callback(),main()
* Global Variables: NONE
'''
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from hb_interfaces.msg import Pose2D, Poses2D
from rclpy.qos import qos_profile_sensor_data # changed
import time

class PoseDetector(Node):

    """
    ------------------------------------------------------------
    Function: __init__

    Input:
        None

    Output:
        None

    Logic Explanation:

        1. Initialize ROS2 node named 'localization_node'.

        2. Configure CvBridge for ROS–OpenCV image conversion.

        3. Define marker sizes for bots and crates.

        4. Create subscribers for camera image and camera info.

        5. Create publishers for bot and crate poses.

        6. Initialize camera calibration placeholders and
           homography matrix.

        7. Define arena world coordinates using corner marker IDs.

        8. Configure ArUco detector parameters for improved accuracy.

        9. Set camera height and marker heights for parallax correction.

        10. Prepare validation parameters and logging.

    Example:
        PoseDetector()
    """
    def __init__(self):
        super().__init__('localization_node')

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()

        # ---------- PARAMETERS ----------
        self.crates_marker_length = 0.045
        self.bots_marker_length = 0.06

        # ---------- TOPICS ----------
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 2)
        # self.image_sub = self.create_subscription(Image,  "/camera/image_raw", self.image_callback, qos_profile_sensor_data )  # Replaces the '10' queue size)
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info_callback, 2)

        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 2)
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 2)

        # ---------- CAMERA PARAMETERS ----------
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        # Add these variables to hold the lookup tables
        self.mapx = None 
        self.mapy = None

        # ---------- HOMOGRAPHY MATRICES ----------
        self.H_matrix = None
        self.homography_computed = False  ## changed

        # ---------- WORLD COORDINATE MAPPING ----------
        # Corner marker IDs (in order: top-left, top-right, bottom-right, bottom-left)
        self.corner_ids = [1, 3, 7, 5]

        # World coordinates for corner markers (in mm)
        # Based on arena image: origin at top-left, X right, Y down
        self.world_matrix = np.array([
            [0.0, 0.0],           # ID 1: top-left (origin)
            [2438.4, 0.0],        # ID 3: top-right
            [2438.4, 2438.4],     # ID 7: bottom-right
            [0.0, 2438.4]         # ID 5: bottom-left
        ], dtype=np.float32)

        # Bot marker IDs
        self.bot_ids = [0, 2, 4]

        # Crate marker IDs
        self.crate_ids = [14, 26, 23, 12, 21, 30]

        # ---------- ARUCO SETUP ----------
        # self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Optimize detection parameters
        self.aruco_params.minMarkerPerimeterRate = 0.01
        self.aruco_params.maxMarkerPerimeterRate = 8.0
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        
        self.aruco_params.cornerRefinementWinSize = 5

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Camera and marker heights for parallax correction
        self.camera_height = 2438.4  # mm
        self.bot_height = 158.0  # Bot marker height above ground level
        self.crate_height = 50.0  # Crate markers height above ground level

        # Known bot position for validation
        self.known_bot_position = (1219.2, 219.2)

        self.get_logger().info('PoseDetector initialized')

    """
    ------------------------------------------------------------
    Function: camera_info_callback

    Input:
        msg (CameraInfo) :
            Camera intrinsic calibration parameters.

    Output:
        None

    Logic Explanation:

        1. Extract camera matrix and distortion coefficients.

        2. Store calibration parameters for image undistortion.

        3. Ensure calibration is processed only once.

        4. Enable downstream homography computation.

    Example:
        Triggered by '/camera/camera_info'.
    """
    

    def camera_info_callback(self, msg):
        """Extract camera calibration parameters"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            
            # Pre-compute the mapping for remap
            self.mapx, self.mapy = cv2.initUndistortRectifyMap(
                self.camera_matrix, 
                self.dist_coeffs, 
                None, 
                self.camera_matrix, 
                (msg.width, msg.height), 
                cv2.CV_32FC1
            )
            
            self.camera_info_received = True
            self.get_logger().info('Camera calibration parameters received and maps generated')
    
    """
    ------------------------------------------------------------
    Function: pixel_to_world

    Input:
        pixel_x (float), pixel_y (float) :
            Pixel coordinates in image frame.

    Output:
        tuple (world_x, world_y)

    Logic Explanation:

        1. Validate that homography matrix exists.

        2. Transform pixel coordinates to world coordinates
           using perspective transformation.

        3. Return mapped arena coordinates in millimeters.

        4. Handle transformation errors safely.

    Example:
        self.pixel_to_world(px, py)
    """
    def pixel_to_world(self, pixel_x, pixel_y):
        """Convert pixel coordinates to world coordinates using homography"""
        if self.H_matrix is None:
            return None, None

        try:
            pixel_point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
            world_point = cv2.perspectiveTransform(pixel_point, self.H_matrix)
            return world_point[0][0][0], world_point[0][0][1]
        except Exception as e:
            self.get_logger().error(f'Error in pixel_to_world: {e}')
            return None, None
    
    """
    ------------------------------------------------------------
    Function: calculate_marker_orientation

    Input:
        corner (array) :
            Detected marker corner coordinates.

    Output:
        float :
            Yaw angle in radians.

    Logic Explanation:

        1. Extract top edge vector of marker.

        2. Compute orientation using arctangent.

        3. Return yaw angle in radians.

        4. Provides orientation for robot heading estimation.

    Example:
        self.calculate_marker_orientation(corner)
    """
    def calculate_marker_orientation(self, corner):
        """Calculate yaw angle from marker corners"""
        try:
            corners = corner[0]
            # Vector from corner 0 to corner 1 (top edge)
            vec_top = corners[1] - corners[0]
            angle_rad = np.arctan2(vec_top[1], vec_top[0])
            return angle_rad
        except Exception as e:
            self.get_logger().error(f'Error calculating orientation: {e}')
            return 0.0

    """
    ------------------------------------------------------------
    Function: apply_parallax_correction

    Input:
        world_x (float), world_y (float)
        marker_id (int)

    Output:
        tuple (corrected_x, corrected_y)

    Logic Explanation:

        1. Determine marker height based on ID.

        2. Skip correction for ground-level markers.

        3. Compute displacement relative to camera center.

        4. Apply height-based parallax scaling.

        5. Return corrected world coordinates.

        6. Improves spatial accuracy for elevated markers.

    Example:
        self.apply_parallax_correction(x, y, marker_id)
    """ 
    def apply_parallax_correction(self, world_x, world_y, marker_id):
        """Apply parallax correction for elevated markers"""
        try:
            # Determine marker height
            if marker_id in self.bot_ids:
                marker_height = self.bot_height
            elif marker_id in self.crate_ids:
                marker_height = self.crate_height
            elif marker_id in self.corner_ids:
                marker_height = 0.0
            else:
                marker_height = 0.0

            # Skip correction for ground-level markers
            if marker_height == 0.0:
                return world_x, world_y

            # Camera at center of arena
            camera_x = 2438.4 / 2.0
            camera_y = 2438.4 / 2.0

            # Calculate displacement
            dx = world_x - camera_x
            dy = world_y - camera_y

            # Apply parallax correction
            parallax_factor = marker_height / self.camera_height
            corrected_x = world_x - (dx * parallax_factor)
            corrected_y = world_y - (dy * parallax_factor)

            return corrected_x, corrected_y

        except Exception as e:
            self.get_logger().error(f'Error in parallax correction: {e}')
            return world_x, world_y
    
    """
    ------------------------------------------------------------
    Function: image_callback

    Input:
        msg (Image) :
            Raw camera image.

    Output:
        None

    Logic Explanation:

        1. Ensure camera calibration is available.

        2. Convert ROS image to OpenCV format.

        3. Undistort image using calibration parameters.

        4. Enhance contrast using CLAHE.

        5. Detect ArUco markers.

        6. Identify corner markers and compute homography.

        7. Convert detected marker pixel centers
           to world coordinates.

        8. Apply parallax correction and compute orientation.

        9. Categorize markers into bots and crates.

        10. Publish corresponding pose arrays.

        11. Visualize detection results for debugging.

        12. Maintains real-time localization for
            multi-robot coordination.

    Example:
        Triggered by '/camera/image_raw'.
    """
    def image_callback(self, msg):
        start_time = time.perf_counter()
        """Process image and detect markers"""
        if not self.camera_info_received:
            self.get_logger().warn('Waiting for camera info...', throttle_duration_sec=2.0)
            return

        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Undistort image
            #
            undistorted = cv2.remap(cv_image, self.mapx, self.mapy, cv2.INTER_LINEAR)
            # Convert to grayscale
            gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

            # Apply CLAHE for better contrast
            clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)

        
            SCALE = 0.70
            
            # Shrink image by the SCALE factor
            small_enhanced = cv2.resize(enhanced, (0, 0), fx=SCALE, fy=SCALE, interpolation=cv2.INTER_LINEAR)
            corners, ids, rejected = self.detector.detectMarkers(small_enhanced)
            
            # ---> Scale corners back to 1080p using the exact inverse! <---
            if ids is not None:
                corners = list(corners)
                for i in range(len(corners)):
                    # Multiply by (1.0 / SCALE) to perfectly restore original coordinates
                    corners[i] = corners[i] * (1.0 / SCALE)

            if ids is None:
                self.get_logger().warn('No markers detected', throttle_duration_sec=2.0)
                return

            ids = ids.flatten()

            # Log detected markers
            self.get_logger().info(
                f'Detected {len(ids)} markers: {sorted(ids.tolist())}',
                throttle_duration_sec=2.0
            )

            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(undistorted, corners, ids)

            # Build homography from corner markers
            corner_pixels = {}
            for i, marker_id in enumerate(ids):
                if marker_id in self.corner_ids:
                    corner_pts = corners[i][0]
                    corner_pixels[marker_id] = corner_pts

            # Compute homography if we have all 4 corners
            if len(corner_pixels) == 4 and not self.homography_computed:
                # Use specific corners of each marker for precise mapping
                # ID 1 (top-left): use top-left corner (index 0)
                # ID 3 (top-right): use top-right corner (index 1)
                # ID 7 (bottom-right): use bottom-right corner (index 2)
                # ID 5 (bottom-left): use bottom-left corner (index 3)
                pixel_matrix = np.array([
                    corner_pixels[1][0],  # ID 1: top-left corner
                    corner_pixels[3][1],  # ID 3: top-right corner
                    corner_pixels[7][2],  # ID 7: bottom-right corner
                    corner_pixels[5][3]   # ID 5: bottom-left corner
                ], dtype=np.float32)

                # Compute homography
                self.H_matrix, _ = cv2.findHomography(
                    pixel_matrix,
                    self.world_matrix,
                    method=cv2.RANSAC,
                    ransacReprojThreshold=3.0
                )

                if self.H_matrix is not None:
                    self.homography_computed = True
                    self.get_logger().info('Homography computed successfully', once=True)

                    # Validate accuracy
                    world_points = self.world_matrix.reshape(-1, 1, 2)
                    projected = cv2.perspectiveTransform(
                        world_points,
                        np.linalg.inv(self.H_matrix)
                    )
                    errors = np.linalg.norm(pixel_matrix - projected.reshape(-1, 2), axis=1)
                    mean_error = np.mean(errors)
                    self.get_logger().info(
                        f'Homography error: {mean_error:.2f}px',
                        once=True
                    )
            else:
                missing = set(self.corner_ids) - set(corner_pixels.keys())
                self.get_logger().warn(
                    f'Corner markers: {len(corner_pixels)}/4. Missing: {missing}',
                    throttle_duration_sec=2.0
                )

            if self.H_matrix is None:
                return

            # Process all markers
            bot_poses = Poses2D()
            crate_poses = Poses2D()

            for i, marker_id in enumerate(ids):
                # Skip corner markers
                if marker_id in self.corner_ids:
                    continue

                # Get marker center in pixels
                corner_pts = corners[i][0]
                center_x = np.mean(corner_pts[:, 0])
                center_y = np.mean(corner_pts[:, 1])

                # Convert to world coordinates
                world_x, world_y = self.pixel_to_world(center_x, center_y)

                if world_x is None or world_y is None:
                    continue

                # Apply parallax correction
                world_x, world_y = self.apply_parallax_correction(world_x, world_y, marker_id)

                # Calculate orientation
                yaw_rad = self.calculate_marker_orientation(corners[i])

                # Create pose message
                pose = Pose2D()
                pose.id = int(marker_id)
                pose.x = float(world_x)
                pose.y = float(world_y)
                pose.w = float(yaw_rad)  # Store as radians

                display_text = f"ID:{marker_id} X:{world_x:.1f} Y:{world_y:.1f} W:{math.degrees(yaw_rad):.1f}deg"



                # Categorize and publish
                if marker_id in self.bot_ids:
                    bot_poses.poses.append(pose)

                    # Validation
                    error_x = abs(world_x - self.known_bot_position[0])
                    error_y = abs(world_y - self.known_bot_position[1])
                    total_error = np.sqrt(error_x**2 + error_y**2)

                    self.get_logger().info(
                        f'Bot {marker_id}: x={world_x:.1f}, y={world_y:.1f}, '
                        f'yaw={math.degrees(yaw_rad):.1f}° | Error: {total_error:.1f}mm',
                        throttle_duration_sec=1.0
                    )

                    # Draw on image
                    cv2.putText(undistorted, display_text,
                               (int(center_x) + 5, int(center_y) - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                elif marker_id in self.crate_ids:
                    crate_poses.poses.append(pose)

                    self.get_logger().info(
                        f'Crate {marker_id}: x={world_x:.1f}, y={world_y:.1f}, '
                        f'yaw={math.degrees(yaw_rad):.1f}°',
                        throttle_duration_sec=1.0
                    )

                    # Draw on image
                    cv2.putText(undistorted, display_text,
                               (int(center_x) + 5, int(center_y) - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Publish poses
            if len(bot_poses.poses) > 0:
                self.bot_poses_pub.publish(bot_poses)

            if len(crate_poses.poses) > 0:
                self.crate_poses_pub.publish(crate_poses)

            # Display image
            cv2.imshow('Detected Markers', undistorted)
            cv2.waitKey(1)
            end_time = time.perf_counter()
            processing_time = (end_time - start_time) * 1000
            self.get_logger().info(f"Image processing time: {processing_time:.2f}ms")

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
     # Ensure this is at the top of your file

    
"""
------------------------------------------------------------
Function: main

Input:
    args :
        Optional ROS2 command-line arguments.

Output:
    None

Logic Explanation:

    1. Initialize ROS2 communication layer.

    2. Create PoseDetector node instance.

    3. Spin node to process image and camera callbacks.

    4. On shutdown, destroy node and close OpenCV windows.

Example:
    main()
"""
def main(args=None):
    rclpy.init(args=args)
    pose_detector = PoseDetector()
    try:
        rclpy.spin(pose_detector)
    except KeyboardInterrupt:
        pass
    finally:
        pose_detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()