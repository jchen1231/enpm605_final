import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, StaticTransformBroadcaster
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import tf_transformations
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory
# import gz.transport


class NavigationNode(Node):
    
    def __init__(self):
        super().__init__("navigation_node")
        
        # Setting parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "goals_file",
                    "goal_poses.yaml",
                    ParameterDescriptor(description="File containing goals"),
                ),
                (
                    "camera_pose",
                    [-8.0, 7.0, 0.25, 0.0, 0.0, 1.57], # [x, y, z, roll, pitch, yaw]
                    ParameterDescriptor(description="Pose of camera pointing to cube #1"),
                ),
                (
                    'x',
                    0.0,
                    ParameterDescriptor(description="Where robot starts x, for localization")
                ),
                (
                    'y',
                    2.0,
                    ParameterDescriptor(description="Where robot starts y, for localization")
                ),
            ],
        )
        # Defaults the use
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            self.get_logger().info("Using the passed 'use_sim_time' parameter value")
            pass
        
        # Getting and setting parameters
        self._goals_file = self.get_parameter("goals_file").value
        self._camera_pose = self.get_parameter("camera_pose").value
        self._startx = self.get_parameter('x').value
        self._starty = self.get_parameter('y').value
        
        
        # Load predefined goals
        self._cube2_goals = dict()
        self._final_goals = dict()
        self._load_goals()
        
        # Declare other attributes
        self._MAP_OFFSET_X = 0.0
        self._MAP_OFFSET_Y = 2.0
        self._CUBE_SIZE = 0.5
        self._CAMERA_DEPTH_OFFSET = 0.75
        self._DIST_FROM_CUBE = 1
        self._initial_pose = PoseStamped()
        self._aruco_frame = "cube1_aruco"
        self._world_frame = "map"
        self._camera_frame = "static_camera"
        self._detected_cube2_id = None
        self._detected_cube2_position = list()
        self._detected_cube2_orientation = list()
        self._detected_cube2_circular_direction = None
        self._detected_final_id = None
        self._detected_final_position = list()
        self._detected_final_orientation = list()
        # # Extract the required values for cube 2 goal from parameters
            
        # Navigator
        self._navigator = BasicNavigator()
        # Set the initial pose of the robot
        self._localize()
        
        # Declare flags
        self._statics_broadcasted = False
        self._phase1_complete = False
        self._phase2_complete = False
        self._phase3_complete = False
        
        # Create subscribers
        # For the static camera
        self._aruco_marker_sub1 = self.create_subscription(ArucoMarkers, "/aruco_markers", self._aruco_marker_sub_cb1, 3)
        # For the dynamic camera onbaord Rosbot at cube #1
        self._aruco_marker_sub2 = self.create_subscription(ArucoMarkers, "/camera/aruco_markers", self._aruco_marker_sub_cb2, 3)
        # For the dynamic camera onbaord Rosbot at cube #2
        self._aruco_marker_sub3 = self.create_subscription(ArucoMarkers, "/camera/aruco_markers", self._aruco_marker_sub_cb3, 3)
        # self.create_timer(5,self._aruco_marker_sub_cb3)
        
        # TF system setup
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        # Create timer to periodically check transforms
        # self._listener_timer = self.create_timer(1.0, self._check_transform)
        
        # Start the BasicNavigator action client
        self._init_timer = self.create_timer(5.0, self._initialize_navigation_cb)
        
        # Log
        self.get_logger().info("Successfully initialized navigation_node.")
        
        
    def _load_goals(self) -> None:
        """Load goal positions from YAML file"""
        
        self.get_logger().info("Loading goals from file")

        # If not an absolute path, check in the package share directory
        if not os.path.isabs(self._goals_file):
            try:
                package_dir = get_package_share_directory("final")
                goals_path = os.path.join(package_dir, "config", self._goals_file)
            except Exception as e:
                self.get_logger().warn(f"Failed to locate package directory: {e}")

        # If file doesn't exist, use default values
        if not os.path.exists(goals_path):
            self.get_logger().warn(
                f"Goals file not found at {goals_path}, using default values"
            )
            # Default goals if file not found

            return {}

        # Try to load the file
        try:
            with open(goals_path, "r") as file:
                goals = yaml.safe_load(file)
                if not goals:
                    raise ValueError("Empty or invalid YAML file")
                
                self._cube2_goals = goals["cube2_goal"]
                self._final_goals = goals["final_goal"]
                self.get_logger().info(f"Loaded cube 2 goals from {goals_path}")
                
                for key, value in self._cube2_goals.items():
                    self.get_logger().info(
                        f"ArUco ID:{key} -- (x: {value['position'][0]}, {value['position'][1]})"
                    )
                self.get_logger().info(f"Loaded final goals from {goals_path}")
                
                for key, value in self._final_goals.items():
                    self.get_logger().info(
                        f"ArUco ID:{key} -- (x: {value['position'][0]}, {value['position'][1]})"
                    )
                    
                self.get_logger().info("All goals loaded successfully")

        except Exception as e:
            self.get_logger().error(f"Error loading goals file: {e}")
            # Return empty dict - service will return failure
            return {}         

        
    def _aruco_marker_sub_cb1(self, msg: ArucoMarkers) -> None:
        if self._statics_broadcasted is True:
            return
        
        try:
            # Broadcast the camera frame
            pose_stamped = self._create_pose_stamped(
                self._camera_pose[0] - self._MAP_OFFSET_X, 
                self._camera_pose[1] - self._MAP_OFFSET_Y,
                self._camera_pose[2], 
                self._camera_pose[5], 
                self._world_frame
            )
            transform_stamped = self._create_transform_stamped(pose_stamped, self._camera_frame)
            self._static_tf_broadcaster.sendTransform(transform_stamped)
            self.get_logger().info(f"Successfully broadcasted frame {self._camera_frame}.")
                
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast tf {self._camera_frame}: {str(e)}")
            
        try:
            # Create aruco marker stamped transform
            aruco_pose_stamped = PoseStamped()
            aruco_pose_stamped.header.frame_id = self._camera_frame
            aruco_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            aruco_pose_stamped.pose = msg.poses[0]
            temp = msg.poses[0].position.x
            aruco_pose_stamped.pose.position.x = msg.poses[0].position.z - self._CAMERA_DEPTH_OFFSET
            aruco_pose_stamped.pose.position.z = temp
            aruco_transform_stamped = self._create_transform_stamped(
                aruco_pose_stamped,
                self._aruco_frame,
            )
            
            # Broadcast the transform of aruco marker for cube #1
            self._static_tf_broadcaster.sendTransform(aruco_transform_stamped)
            self.get_logger().info(f"Successfully broadcasted frame {self._aruco_frame}.")
            
            # Update flag
            self._statics_broadcasted = True
            
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast tf {self._aruco_frame}: {str(e)}")   
        

    def _aruco_marker_sub_cb2(self, msg: ArucoMarkers) -> None:
        if self._phase1_complete is False:
            return
        if self._phase2_complete is True:
            return
        
        self._detected_cube2_id = msg.marker_ids[0]
        self._detected_cube2_position = self._cube2_goals[self._detected_cube2_id]['position']
        # self.get_logger().info(f'position {self._detected_cube2_position}')
        self._detected_cube2_orientation = self._cube2_goals[self._detected_cube2_id]['orientation']
        self._detected_cube2_circular_direction = self._cube2_goals[self._detected_cube2_id]['circular_direction']
        self.get_logger().info(f"Detected an aruco marker at cube #1 with the ID: {self._detected_cube2_id}.")
        
        try:
            self._navigate(
                self._detected_cube2_position[0] - self._MAP_OFFSET_X,
                self._detected_cube2_position[1] - self._MAP_OFFSET_Y,
                self._detected_cube2_orientation[2],
                )
            
            # Follow the waypoints
            # self.follow_waypoints()
            
            self.get_logger().info("Navigated to cube #2 successfully")
            
            # Update flag
            self._phase2_complete = True
            
        except Exception as e:
            self.get_logger().error(f"Failed to navigate to cube #2: {str(e)}")
        
        
    def _aruco_marker_sub_cb3(self, msg) -> None:
        if self._phase2_complete is False:
            return
        if self._phase3_complete is True:
            return
        # self._localize()
        # Grabbed final coordinates, but need to circle around the aruco box first
        self._detected_final_id = msg.marker_ids[0]
        self._detected_final_position = self._final_goals[self._detected_final_id]['position']
        self._detected_final_orientation = self._final_goals[self._detected_final_id]['orientation']
        self.get_logger().info(f"Detected an aruco marker at cube #2 with the ID: {self._detected_final_id}.")
        
        try:
            # Make path using current Rosbot pose and pose of cube (which the robot faces)
            rosbot_x = self._detected_cube2_position[0]
            rosbot_y = self._detected_cube2_position[1]
            rosbot_yaw = self._detected_cube2_orientation[2]
            # direction is positive and CCW if 1, direction is negative and CW if 0
            direction = (self._detected_cube2_circular_direction)*2 - 1
            
            # rosbot_x = 3.75
            # rosbot_y = -5.25
            # rosbot_yaw = -1.57
            # direction = -1
            
            radius = (self._CUBE_SIZE/2 + self._DIST_FROM_CUBE)
            cube_x = rosbot_x + radius * np.cos(rosbot_yaw)
            cube_y = rosbot_y + radius * np.sin(rosbot_yaw)
            
            slices = 60
            step_size = np.pi*2/slices*direction
            wp_arr = list()
            
            for i in range(slices):
                x = radius * np.cos(-rosbot_yaw + i * step_size) + cube_x - self._MAP_OFFSET_X
                y = radius * np.sin(-rosbot_yaw + i * step_size) + cube_y - self._MAP_OFFSET_Y
                wp_arr.append([x, y])
            
            # Compute yaw between every two points of traveling along the circle
            final_wp_arr = list()
            for i in range(len(wp_arr)-1):
                x0, y0 = wp_arr[i]
                x, y = wp_arr[i + 1]
                yaw = np.arctan2(y-y0, x-x0)
                final_wp_arr.append([x, y, 0.0, yaw])
            
            # Follow the waypoints
            self._follow_waypoints(final_wp_arr)
            
            self.get_logger().info("Circled around cube #2 successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to circle around cube #2: {str(e)}")
    
        try:
            # Now navigate to final destination
            self._navigate(
                self._detected_final_position[0] - self._MAP_OFFSET_X,
                self._detected_final_position[1] - self._MAP_OFFSET_Y,
                self._detected_final_orientation[2],
                )
            
            self.get_logger().info("Navigated to final destination successfully")
            
            # Update flag
            self._phase3_complete = True
            
        except Exception as e:
            self.get_logger().error(f"Failed to navigate to cube final destination: {str(e)}")
    
    
    def _initialize_navigation_cb(self) -> None:
        if self._statics_broadcasted is False:
            return
        if self._phase1_complete is True:
            return
        
        # Cancel the timer so this only runs once
        self._init_timer.cancel()
        
        try: 
            x, y = self._lookup_tf(self._world_frame, self._aruco_frame)
            
            # Go to a set distance opposing the static_camera
            x += np.cos(self._camera_pose[5]) * (self._CUBE_SIZE + self._DIST_FROM_CUBE)
            y += np.sin(self._camera_pose[5]) * (self._CUBE_SIZE + self._DIST_FROM_CUBE)
            # Rotate 180 from the yaw of statoc_camera
            yaw = self._camera_pose[5] - 3.14
            self._navigate(x, y, yaw)
            
            # Follow the waypoints
            # self.follow_waypoints()
            
            self.get_logger().info("Navigated to cube #1 successfully")
            
            # Update flag
            self._phase1_complete = True
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize navigation: {str(e)}")
            # Try again after a delay by creating a new timer
            self._init_timer = self.create_timer(5.0, self._initialize_navigation_cb)


    def _lookup_tf(self, parent_frame: str, child_frame: str) -> tuple[float,float]:
        try:
            transform = self._tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1),
            )

            # Process the transform data
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            # Log the information
            self.get_logger().info(
                f"Frame {child_frame} is at position [{tx:.3f}, {ty:.3f}, {tz:.3f}] "
                f"relative to {parent_frame}"
            )

            return (tx, ty)

        except Exception as e:
            # If transform isn't available yet or has expired, don't report an error
            if "lookup would require extrapolation" not in str(e):
                self.get_logger().debug(
                    f"Cannot look up transform: {e}"
                    )


    def _localize(self):
        """
        Set the initial pose of the robot.
        """
        # Set the initial pose of the robot
        self._initial_pose.header.frame_id = self._world_frame
        self._initial_pose.header.stamp = self.get_clock().now().to_msg()
        self._initial_pose.pose.position.x = self._startx - self._MAP_OFFSET_X
        self._initial_pose.pose.position.y = self._starty - self._MAP_OFFSET_Y
        self._initial_pose.pose.position.z = 0.0
        self._initial_pose.pose.orientation.x = 0.0
        self._initial_pose.pose.orientation.y = 0.0
        self._initial_pose.pose.orientation.z = 0.0
        self._initial_pose.pose.orientation.w = 1.0
        self._navigator.setInitialPose(self._initial_pose)
        self.get_logger().info("Initial robot pose is set.")

    def _navigate(self, x: float, y: float, yaw: float) -> None:
        """
        Navigate the robot to the goal (x, y).
        """
        # Wait until Nav2 is active
        self._navigator.waitUntilNav2Active()
        
        goal = self._create_pose_stamped(x, y, 0.0, yaw, self._world_frame)
        self._navigator.goToPose(goal)
        self.get_logger().info(f"Attempting to navigate to ({x}, {y})")
        while not self._navigator.isTaskComplete():
            feedback = self._navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback}")
        
        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")

    def _follow_waypoints(self, wp_arr: list[list[float, float, float, float]]) -> None:
        # Wait until Nav2 is active
        self._navigator.waitUntilNav2Active()  
        waypoints = list()
        for wp in wp_arr:
            waypoints.append(self._create_pose_stamped(*wp, self._world_frame))

        self._navigator.followWaypoints(waypoints)
        
        while not self._navigator.isTaskComplete():
            feedback = self._navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback}")

        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")

    def _create_pose_stamped(self, x: float, y: float, z: float, yaw: float, frame_id: str) -> PoseStamped:
        """
        Create a PoseStamped message.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z

        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

        pose_stamped.pose.orientation.x = q_x
        pose_stamped.pose.orientation.y = q_y
        pose_stamped.pose.orientation.z = q_z
        pose_stamped.pose.orientation.w = q_w
        return pose_stamped

    def _create_transform_stamped(self, ps: PoseStamped, child_frame: str) -> TransformStamped:
        """
        Create a TransformStamped message.
        """
        transform_stamped = TransformStamped()
        transform_stamped.header.frame_id = ps.header.frame_id
        transform_stamped.header.stamp = ps.header.stamp
        transform_stamped.child_frame_id = child_frame
        transform_stamped.transform.translation.x = ps.pose.position.x
        transform_stamped.transform.translation.y = ps.pose.position.y 
        transform_stamped.transform.translation.z = ps.pose.position.z
        transform_stamped.transform.rotation.x = ps.pose.orientation.x
        transform_stamped.transform.rotation.y = ps.pose.orientation.y
        transform_stamped.transform.rotation.z = ps.pose.orientation.z
        transform_stamped.transform.rotation.w = ps.pose.orientation.w
        
        return transform_stamped

def main(args=None):
    rclpy.init(args=args)
    try:
        node = NavigationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unhandled exception: {e}")
    finally:    
        node.destroy_node()
        rclpy.shutdown()
