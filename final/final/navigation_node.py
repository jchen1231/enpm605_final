import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, StaticTransformBroadcaster
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor
import tf_transformations
import numpy as np
from custom_interfaces.srv import GetGoal
from rclpy.client import Future


class NavigationNode(Node):
    
    def __init__(self):
        super().__init__("navigation_node")
        
        # Declaring parameters
        self.declare_parameters(
            namespace='', # Code breaks without this parameter declared
            parameters=[

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
        # Defaults the use_sim_time parameter if not passed as a parameter
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            self.get_logger().info("Using the passed 'use_sim_time' parameter value")
            pass
        
        # Getting parameters from ros to python
        self._camera_pose = self.get_parameter("camera_pose").value
        self._startx = self.get_parameter('x').value
        self._starty = self.get_parameter('y').value
        
        # Declare other attributes
        self._MAP_OFFSET_X = 0.0
        self._MAP_OFFSET_Y = 2.0
        self._CUBE_SIZE = 0.5
        self._CAMERA_DEPTH_OFFSET = 0.75
        self._DIST_FROM_CUBE = 1
        self._initial_pose = PoseStamped()
        self._aruco_frame = "cube1_aruco" # FRAME OF FIRST ARUCO MARKER
        self._world_frame = "map" # HIGHEST LEVEL FRAME
        self._camera_frame = "static_camera" # FRAME OF THE BLUE CAMERA
        self._detected_cube2_id = None
        self._detected_cube2_position = list()
        self._detected_cube2_orientation = list()
        self._detected_cube2_circular_direction = None
        self._detected_final_id = None
        self._detected_final_position = list()
        self._detected_final_orientation = list()
            
        # Start the Navigator
        self._navigator = BasicNavigator()
        # Set the initial pose of the robot, only needs to be done once?
        self._localize()
        
        # Declare flags for proper transitioning
        self._statics_broadcasted = False
        self._phase1_complete = False
        self._phase2_service_called = False
        self._phase2_complete = False
        self._phase3_service_called = False
        self._phase3_complete = False
        
        # Create subscribers
        # For the static camera
        self._aruco_marker_sub1 = self.create_subscription(ArucoMarkers, "/aruco_markers", self._aruco_marker_sub_cb1, 3)
        # For the dynamic camera onbaord Rosbot at cube #1
        self._aruco_marker_sub2 = self.create_subscription(ArucoMarkers, "/camera/aruco_markers", self._aruco_marker_sub_cb2, 3)
        # For the dynamic camera onbaord Rosbot at cube #2
        self._aruco_marker_sub3 = self.create_subscription(ArucoMarkers, "/camera/aruco_markers", self._aruco_marker_sub_cb3, 3)
        # self.create_timer(5,self._aruco_marker_sub_cb3)
        
        # TF system setup, one static and one dynamic
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0)) # Static broadcasts should not timeout I think
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        # Start the BasicNavigator action client
        self._init_timer = self.create_timer(5.0, self._initialize_navigation_cb)
        
        # Create goal service clients, one for each goal
        self._get_cube2_goal_cli = self.create_client(
            GetGoal,
            "get_cube2_goal",
        )
        self._get_final_goal_cli = self.create_client(
            GetGoal,
            "get_final_goal",
        )
        
        # Log success
        self.get_logger().info("Successfully initialized navigation_node.")
        
        
    def _aruco_marker_sub_cb1(self, msg: ArucoMarkers) -> None:
        """
        Broadcasts the static transforms of the world_frame to the camera_frame and that of the camera_frame to the cube1_aruco frame.

        :param msg: message from the /aruco_markers topic
        :type msg: ArucoMarkers
        """
        
        # Skips if statics are already broadcasted
        if self._statics_broadcasted is True:
            return
        
        # Two different try blocks to give more specific error logs
        try:
            # Create the PosesTamped from the aruco marker pose info
            pose_stamped = self._create_pose_stamped(
                self._camera_pose[0] - self._MAP_OFFSET_X, # Issue due to mapping
                self._camera_pose[1] - self._MAP_OFFSET_Y, # Issue due to mapping
                self._camera_pose[2], 
                self._camera_pose[5], 
                self._world_frame
            )
            # Create the transformStamped from PoseStamped
            transform_stamped = self._create_transform_stamped(pose_stamped, self._camera_frame)
            # Broadcast the camera frame
            self._static_tf_broadcaster.sendTransform(transform_stamped)
            self.get_logger().info(f"Successfully broadcasted frame {self._camera_frame}.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast tf {self._camera_frame}: {str(e)}")
            
        try:
            # Create aruco marker stamped transform
            aruco_pose_stamped = PoseStamped()
            aruco_pose_stamped.header.frame_id = self._camera_frame
            aruco_pose_stamped.header.stamp = self.get_clock().now().to_msg() # The most recent time is used
            aruco_pose_stamped.pose = msg.poses[0]
            # HERE IS THE SWAPPED X AND Z AXIS ISSUE
            temp = msg.poses[0].position.x
            # Camera appears to overshoot the aruco marker pose
            aruco_pose_stamped.pose.position.x = msg.poses[0].position.z - self._CAMERA_DEPTH_OFFSET
            aruco_pose_stamped.pose.position.z = temp
            # CReate TransformStamped
            aruco_transform_stamped = self._create_transform_stamped(
                aruco_pose_stamped,
                self._aruco_frame,
            )
            
            # Broadcast the transform of aruco marker for cube #1
            self._static_tf_broadcaster.sendTransform(aruco_transform_stamped)
            self.get_logger().info(f"Successfully broadcasted frame {self._aruco_frame}.")
            
            # Update flag only if everyting else ran without issue (except block is not triggered)
            self._statics_broadcasted = True
            
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast tf {self._aruco_frame}: {str(e)}")   
        

    def _initialize_navigation_cb(self) -> None:
        """
        Starts the navigation process upon the first aruco marker detection.
        """
        # Skip if the static transforms have not been broadcasted
        if self._statics_broadcasted is False:
            return
        # skip if phase 1 is already complete
        if self._phase1_complete is True:
            return
        
        # Cancel the timer so this only runs once
        self._init_timer.cancel()
        
        try:
            # Look up the transform from map to cube1_aruco
            x, y = self._lookup_tf(self._world_frame, self._aruco_frame)
            
            # Go to a set distance opposing the static_camera
            x += np.cos(self._camera_pose[5]) * (self._CUBE_SIZE + self._DIST_FROM_CUBE)
            y += np.sin(self._camera_pose[5]) * (self._CUBE_SIZE + self._DIST_FROM_CUBE)
            # Rotate 180 from the yaw of static_camera
            yaw = self._camera_pose[5] - 3.14
            
            # Start navigation to cube #1
            self._navigate(x, y, yaw)
            self.get_logger().info("Navigated to cube #1 successfully")
            
            # Update flag
            self._phase1_complete = True
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize navigation: {str(e)}")
            # Try again after a delay by creating a new timer if failure
            self._init_timer = self.create_timer(5.0, self._initialize_navigation_cb)


    def _aruco_marker_sub_cb2(self, msg: ArucoMarkers) -> None:
        """
        Starts phase 2, which is processing the newly detected aruco marker at cube #1 and going to cube #2.

        :param msg: message from the /camera/aruco_markers topic
        :type msg: ArucoMarkers
        """
        # skips if the first phase is not complete
        if self._phase1_complete is False:
            return
        # skips if the second phase is already completed
        if self._phase2_complete is True:
            return
        
        try:
            # Skips if the service has been called once already, to not spam service if it is not active or waiting for response
            if self._phase2_service_called is False:
                # If the service is not active, allows for this whole function to run again
                if not self._get_cube2_goal_cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warn(
                        f"{self._get_cube2_goal_cli.srv_name} service unavailable!"
                    )
                # If the service is active, update the flag
                else:
                    self._detected_cube2_id = msg.marker_ids[0]
                    request = GetGoal.Request()
                    request.id = msg.marker_ids[0]
                    # Call service asynchronously
                    future = self._get_cube2_goal_cli.call_async(request)
                    # add a callback response to process the service response
                    future.add_done_callback(
                        self._aruco_marker_sub_cb2_cli_cb
                    )
                    # Update flag only if everyting else ran without issue (except block is not triggered)
                    self._phase2_service_called = True
                    
        except Exception as e:
            self.get_logger().error(f"Error calling service {self._get_cube2_goal_cli.srv_name}: {str(e)}")   
        
        
    def _aruco_marker_sub_cb2_cli_cb(self, future: Future):
        """
        Process the response returned by the cube2_goal service.
        :param future: future returned by the cube2_goal service
        :type future: Future
        """
        # Resets the flag if the future returned is None
        if future is None:
            self._phase2_service_called = False
            return
        try:
            # Otherwise, extract the results
            result = future.result()
            # If the 
            if result.success is True:
                # Extracct the goal parameters for cube #2
                self._detected_cube2_position = result.position
                self._detected_cube2_orientation = result.orientation
                self._detected_cube2_circular_direction = result.circular_direction
                self.get_logger().info(f"Detected an aruco marker at cube #1 with the ID: {self._detected_cube2_id}.")
                
                # Navigate to the correct goal position accordingly
                self._navigate(
                    self._detected_cube2_position[0] - self._MAP_OFFSET_X,
                    self._detected_cube2_position[1] - self._MAP_OFFSET_Y,
                    self._detected_cube2_orientation[2],
                    )
                self.get_logger().info("Navigated to cube #2 successfully")

                # Update flag only if everyting else ran without issue (except block is not triggered)
                self._phase2_complete = True
            else:
                # Resets the flag if the service call processing was unsuccessful
                self._phase2_service_called = False
                
        except Exception as e:
            self.get_logger().error(f"Failed to navigate to cube #2: {str(e)}")
        
        
    def _aruco_marker_sub_cb3(self, msg: ArucoMarkers) -> None:
        """
        Starts phase 3, which is processing the newly detected aruco marker at cube #2 and starting the circling.

        :param msg: message from the topic /camera/aruco_markers
        :type msg: ArucoMarkers
        """
        # Skip if phase 2 is not complete
        if self._phase2_complete is False:
            return
        # Skip if phase 3 is already completed
        if self._phase3_complete is True:
            return
        
        try:
            # Skip if the serice has been called once already
            if self._phase3_service_called is False:
                # Make sure the service is available
                if not self._get_final_goal_cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warn(
                        f"{self._get_final_goal_cli.srv_name} service unavailable!"
                    )
                else:
                    # send the asynchronous request
                    self._detected_final_id = msg.marker_ids[0]
                    request = GetGoal.Request()
                    request.id = msg.marker_ids[0]
                    future = self._get_final_goal_cli.call_async(request)
                    # Attach a callback to start circling and navigating to the final pose
                    future.add_done_callback(
                        self._aruco_marker_sub_cb3_cli_cb
                    )
                    # If everything runs wihtout issues, then set flag to true
                    self._phase3_service_called = True
                    
        except Exception as e:
            self.get_logger().error(f"Error calling service {self._get_final_goal_cli.srv_name}: {str(e)}")   
        
        
    def _aruco_marker_sub_cb3_cli_cb(self, future: Future):
        """
        Callback function to initiate circling if the final goal can be processed from the message returned from aruco node.

        :param future: The response from the service call
        :type future: Future
        """
        # skip everything and reset flag if the service response is None
        if future is None:
            self._phase3_service_called = False
            return
        
        try:
        # Process the future
            result = future.result()
            # Make sure the service call processing was a success
            if result.success is True:
                # Grabbed final coordinates, but need to circle around the aruco box first
                self._detected_final_position = result.position
                self._detected_final_orientation = result.orientation
                self.get_logger().info(f"Detected an aruco marker at cube #2 with the ID: {self._detected_final_id}.")
                
                # Make path using current Rosbot and cube pose, which the robot faces
                rosbot_x = self._detected_cube2_position[0]
                rosbot_y = self._detected_cube2_position[1]
                rosbot_yaw = self._detected_cube2_orientation[2]
                # direction is positive and CCW if circular_direction==1, direction is negative and CW if circular_direction==0
                direction = (self._detected_cube2_circular_direction)*2 - 1
                # Find the radius of the circle
                radius = (self._CUBE_SIZE/2 + self._DIST_FROM_CUBE)
                cube_x = rosbot_x + radius * np.cos(rosbot_yaw)
                cube_y = rosbot_y + radius * np.sin(rosbot_yaw)
                # Make the circle in 60 steps
                slices = 60
                step_size = np.pi*2/slices*direction
                wp_arr = list()
                
                # Find the new x and y coordinate based on the new angle on the unit circle and the fixed radius
                for i in range(slices):
                    x = radius * np.cos(rosbot_yaw + 3.14 + i * step_size) + cube_x - self._MAP_OFFSET_X
                    y = radius * np.sin(rosbot_yaw + 3.14 + i * step_size) + cube_y - self._MAP_OFFSET_Y
                    wp_arr.append([x, y])
                
                # Compute yaw between every two points of traveling along the circle
                final_wp_arr = list()
                for i in range(len(wp_arr)-1): # stay in bounds of list
                    x0, y0 = wp_arr[i] # before
                    x, y = wp_arr[i + 1] # after
                    yaw = np.arctan2(y-y0, x-x0)
                    # need to have z==0.0 because I reuse PoseStamped generation functions
                    final_wp_arr.append([x, y, 0.0, yaw])
                
                # Follow the waypoints
                self._follow_waypoints(final_wp_arr)
                
                self.get_logger().info("Circled around cube #2 successfully")
            else:
                # Resets the flag if the service call processing was unsuccessful
                self._phase3_service_called = False
                return
                
        except Exception as e:
            self.get_logger().error(f"Failed to circle around cube #2: {str(e)}")

        # different try block for a different phase
        try:
            # Now navigate to final destination using extracted info from earlier
            self._navigate(
                self._detected_final_position[0] - self._MAP_OFFSET_X,
                self._detected_final_position[1] - self._MAP_OFFSET_Y,
                self._detected_final_orientation[2],
                )
            
            self.get_logger().info("Navigated to final destination successfully")
            
            # Update flag only if everything ran successfully
            self._phase3_complete = True
            
        except Exception as e:
            self.get_logger().error(f"Failed to navigate to cube final destination: {str(e)}")


    def _lookup_tf(self, parent_frame: str, child_frame: str) -> tuple[float,float]:
        """
        Returns just the x and y of the child frame origin in the parent frame.

        :param parent_frame: name of the parent frame
        :type parent_frame: str
        :param child_frame: name of the child frame
        :type child_frame: str
        :return: the x and y coordinates to travel to
        :rtype: tuple[float,float]
        """
        try:
            # Look into the buffer for the full transform
            transform = self._tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rclpy.time.Time(), # Has to be a rclpy.time.Time() object or transform breaks
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
        Set the initial pose of the robot, with offsets.
        """
        # not using PoseTransformed bc quaternions
        self._initial_pose = self._create_pose_stamped(
            self._startx - self._MAP_OFFSET_X,
            self._starty - self._MAP_OFFSET_Y,
            0.0,
            0.0,
            self._world_frame,
        )
        # self._initial_pose.header.frame_id = self._world_frame
        # self._initial_pose.header.stamp = self.get_clock().now().to_msg()
        # self._initial_pose.pose.position.x = 
        # self._initial_pose.pose.position.y = 
        # self._initial_pose.pose.position.z = 0.0
        # self._initial_pose.pose.orientation.x = 0.0
        # self._initial_pose.pose.orientation.y = 0.0
        # self._initial_pose.pose.orientation.z = 0.0
        # self._initial_pose.pose.orientation.w = 1.0
        # Set the initial pose of the robot
        self._navigator.setInitialPose(self._initial_pose)
        self.get_logger().info("Initial robot pose is set.")


    def _navigate(self, x: float, y: float, yaw: float) -> None:
        """
        Navigate the robot to the goal (x, y).

        :param x: goal x-coordinate of the robot
        :type x: float
        :param y: goal y-coordinate of the robot
        :type y: float
        :param yaw: goal yaw of the robot
        :type yaw: float
        """
        # Wait until Nav2 is active
        self._navigator.waitUntilNav2Active()
        # Create the desination pose
        goal = self._create_pose_stamped(x, y, 0.0, yaw, self._world_frame)
        # Send a NavToPose action request without behavior tree
        self._navigator.goToPose(goal)
        self.get_logger().info(f"Attempting to navigate to ({x}, {y})")
        # Receving feedback from the indirect action call
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
        """
        Navigate by following a list or array of poses.

        :param wp_arr: list of poses with (x, y, z, yaw)
        :type wp_arr: list[list[float, float, float, float]]
        """
        # Wait until Nav2 is active
        self._navigator.waitUntilNav2Active()  
        waypoints = list()
        for wp in wp_arr:
            # Easy unpack and send to create PoseStamped
            waypoints.append(self._create_pose_stamped(*wp, self._world_frame))

        # Send a FollowWaypoints action request.
        self._navigator.followWaypoints(waypoints)
        
        # Receving feedback from the indirect action call
        while not self._navigator.isTaskComplete():
            feedback = self._navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback}")

        # Get final result of the action call
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

        :param x: x coord of the pose
        :type x: float
        :param y: y coord of the pose
        :type y: float
        :param z: z coord of the pose
        :type z: float
        :param yaw: yaw of the pose
        :type yaw: float
        :param frame_id: frame id for header
        :type frame_id: str
        :return: return the assembled PoseStamped
        :rtype: PoseStamped
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z

        # PoseStamped uses quaternions, so must convert
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

        pose_stamped.pose.orientation.x = q_x
        pose_stamped.pose.orientation.y = q_y
        pose_stamped.pose.orientation.z = q_z
        pose_stamped.pose.orientation.w = q_w
        
        return pose_stamped


    def _create_transform_stamped(self, ps: PoseStamped, child_frame: str) -> TransformStamped:
        """
        Create a TransformStamped message.

        :param ps: The pose of the child frame in the parent frame
        :type ps: PoseStamped
        :param child_frame: child frame name
        :type child_frame: str
        :return: the assembled transformed pose
        :rtype: TransformStamped
        """
        transform_stamped = TransformStamped()
        transform_stamped.header.frame_id = ps.header.frame_id # parent frame
        transform_stamped.header.stamp = ps.header.stamp
        transform_stamped.child_frame_id = child_frame # child frame
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
