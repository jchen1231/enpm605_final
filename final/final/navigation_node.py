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
        
        # Load predefined goals
        self._cube2_goals = dict()
        self._final_goals = dict()
        self._load_goals()
        
        # Declare other attributes
        self._initial_pose = PoseStamped()
        self._aruco_frame = "cube1_aruco"
        self._world_frame = "map"
        self._camera_frame = "camera/camera_link/camera"
        self._detected_cube2_id = None
        self._detected_cube2_position = list()
        self._detected_cube2_orientation = list()
        self._detected_final_id = None
        self._detected_final_position = list()
        self._detected_final_orientation = list()
        
        # Navigator
        self._navigator = BasicNavigator()
        
        # Declare flags
        self._statics_broadcasted = False
        self._cube2_detected = False
        
        # Create subscribers
        self._aruco_marker_sub1 = self.create_subscription(ArucoMarkers, "/aruco_markers", self._aruco_marker_sub_cb1, 3)
        # self._aruco_marker_sub2 = self.create_subscription(ArucoMarkers, "/rosbot/aruco_markers",3, self._aruco_marker_sub_cb2)
        

        # TF system setup
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        pose_stamped = self._create_pose_stamped(-8.0, 8.0, 0.25, 1.57, self._world_frame)
        transform_stamped = self._create_transform_stamped(pose_stamped, self._camera_frame)
        self._static_tf_broadcaster.sendTransform(transform_stamped)
        
        # self._static_tf_broadcaster.sendTransform(self._create_transform_stamped(pose_stamped, "odom"))
        
        self.get_logger().info(f"Successfully broadcasted the static frame {self._camera_frame}.")
        
        # Create timer to periodically check transforms
        # self._listener_timer = self.create_timer(1.0, self._check_transform)
        
        # Start the BasicNavigator action client
        self._init_timer = self.create_timer(5.0, self._initialize_navigation_cb)
        
        # Debug log
        self.get_logger().info("Successfully initialized navigation_node.")
        
        
    def _load_goals(self) -> None:
        """Load goal positions from YAML file"""
        
        self.get_logger().info("Loading goals from file")
        
        # Try to find the goals file in various locations
        goals_path = self.get_parameter("goals_file").value

        # If not an absolute path, check in the package share directory
        if not os.path.isabs(goals_path):
            try:
                package_dir = get_package_share_directory("final")
                goals_path = os.path.join(package_dir, "config", goals_path)
            except Exception as e:
                self.get_logger().warn(f"Failed to locate package directory: {e}")

        # If file doesn't exist, use default values
        if not os.path.exists(goals_path):
            self.get_logger().warn(
                f"Goals file not found at {goals_path}, using default values"
            )
            # # Default goals if file not found
            # goals = {
            #     "2": {
            #         "position": {"x": 1.0, "y": 0.0, "z": 0.0},
            #         "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            #     },
            #     "4": {
            #         "position": {"x": 0.0, "y": 1.0, "z": 0.0},
            #         "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            #     },
            #     "5": {
            #         "position": {"x": -1.0, "y": 0.0, "z": 0.0},
            #         "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            #     },
            #     "6": {
            #         "position": {"x": 4.0, "y": -2.0, "z": 0.0},
            #         "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            #     },
            # }
            # return goals
            return {}

        # Try to load the file
        try:
            with open(goals_path, "r") as file:
                goals = yaml.safe_load(file)
                if not goals:
                    raise ValueError("Empty or invalid YAML file")

                
                cube2_g = goals["cube2_goal"]
                final_g = goals["final_goal"]
                self.get_logger().info(f"Loaded cube 2 goals from {goals_path}")
                for goal in cube2_g:
                    self.get_logger().info(
                        f"ArUco ID:{goal['id']} -- (x: {goal['position'][0]}, {goal['position'][1]})"
                    )
                    self._cube2_goals[goal['id']] = goal
                self.get_logger().info(f"Loaded final goals from {goals_path}")
                for goal in final_g:
                    self.get_logger().info(
                        f"ArUco ID:{goal['id']} -- (x: {goal['position'][0]}, {goal['position'][1]})"
                    )
                    self._final_goals[goal['id']] = goal
                    
                self.get_logger().info("All goals loaded successfully")

        except Exception as e:
            self.get_logger().error(f"Error loading goals file: {e}")
            # Return empty dict - service will return failure
            return {}    
        
        
    def _aruco_marker_sub_cb1(self, msg: ArucoMarkers) -> None:
        if self._statics_broadcasted is True:
            return
        try:
            
            # Extract the required values for cube 2 goal from parameters
            frame_id = msg.header.frame_id
            detected_cube2_id = msg.marker_ids[0]
            detected_cube2_position = self._cube2_goals[detected_cube2_id]["position"]
            detected_cube2_orientation = self._cube2_goals[detected_cube2_id]["orientation"]
            
            # Broadcast the aruco marker frame
            transform_stamped = self._create_transform_stamped(
                self._create_pose_stamped(
                    *detected_cube2_position, 
                    0.0, 
                    detected_cube2_orientation[2],
                    frame_id
                    ),
                self._aruco_frame
            )
            self.get_logger().error(f"{transform_stamped}")
            
            # Broadcast the transform
            self._static_tf_broadcaster.sendTransform(transform_stamped)
            
            # Update flag
            self._statics_broadcasted = True
            
            # Log
            # self.get_logger().info("Successfully broadcasted transform for cube #1.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast tf: {str(e)}")
            # Try again after a delay by creating a new timer        
        

        
    # def _aruco_marker1_sub_cb2(self, msg: ArucoMarkers) -> None:
    #     goal_index = self._final_goal_ids.index(msg.marker_ids)
    #     # Extract the required values from parameters
    #     self._detected_final_id = msg.marker_ids
    #     self._detected_final_position = self._final_goal_positions[goal_index]
    #     self._detected_final_orientation = self._final_goal_orientations[goal_index]  
        
    
    def _initialize_navigation_cb(self) -> None:
        if self._statics_broadcasted is False:
            return

        # No longer need this subscriber
        # self._aruco_marker_sub1.destroy()

        # Cancel the timer so this only runs once
        self._init_timer.cancel()
        
        try: 
            # Set the initial pose of the robot
            self._localize()
            
            x, y = self._lookup_tf(self._world_frame, self._aruco_frame)
            
            # Go to a goal
            self._navigate(x, y)
            
            # Follow the waypoints
            # self.follow_waypoints()
            
            self.get_logger().info("Navigation initialized successfully")
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
                rclpy.duration.Duration(seconds=1),
            )

            # Process the transform data
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            # Calculate distance from origin of base frame to marker
            distance = np.sqrt(tx * tx + ty * ty + tz * tz)

            # Log the information
            self.get_logger().info(
                f"Cube with frame {child_frame} is at position [{tx:.3f}, {ty:.3f}, {tz:.3f}] "
                f"relative to {parent_frame} (distance: {distance:.3f}m)"
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
        self._initial_pose.header.stamp = self._navigator.get_clock().now().to_msg()
        self._initial_pose.pose.position.x = 0.0
        self._initial_pose.pose.position.y = 0.0
        self._initial_pose.pose.position.z = 0.0
        self._initial_pose.pose.orientation.x = 0.0
        self._initial_pose.pose.orientation.y = 0.0
        self._initial_pose.pose.orientation.z = 0.0
        self._initial_pose.pose.orientation.w = 1.0
        self._navigator.setInitialPose(self._initial_pose)
        self.get_logger().info("Initial robot pose is set.")

    def _navigate(self, x: float, y: float):
        """
        Navigate the robot to the goal (x, y).
        """
        # Wait until Nav2 is active
        self._navigator.waitUntilNav2Active() 
        
        goal = self._create_pose_stamped(x, y, 0.0, 0.0, self._world_frame)
        
        self._navigator.goToPose(goal)
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
