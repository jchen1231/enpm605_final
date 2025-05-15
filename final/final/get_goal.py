import rclpy
from rclpy.node import Node
from custom_interfaces.srv import GetGoal
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class GoalProviderService(Node):
    def __init__(self):
        super().__init__("get_goal")
        # Declaring parameters
        self.declare_parameter("goals_file", "goal_poses.yaml")
        
        # Parameters
        self._goals_file = self.get_parameter("goals_file").value

        # Load predefined goals
        self._cube2_goals = dict()
        self._final_goals = dict()
        self._load_goals()
        self.get_logger().info("Loaded predefined goals")

        # Create GetGoal service for cube #2
        self._get_goal_srv = self.create_service(
            GetGoal, "get_cube2_goal", self._get_cube2_goal_request
        )
        
        # Create GetGoal service for final pose
        self._get_goal_srv = self.create_service(
            GetGoal, "get_final_goal", self._get_final_goal_request
        )

        self.get_logger().info("get_goal node initialized")
        
       
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
            self._cube2_goals = {
                2: {
                    "position": [3.75, -7.75],
                    "orientation": [0.0, 0.0, 1.57],
                    "circular_direction": 0,
                    },
                4: {
                    "position": [2.75, -6.5],
                    "orientation": [0.0, 0.0, 0.0],
                    "circular_direction": 0,
                    },
                5: {
                    "position": [3.75, -5.25],
                    "orientation": [0.0, 0.0, -1.57],
                    "circular_direction": 0,
                    },
                6: {
                    "position": [5.0, -6.5],
                    "orientation": [0, 0, 3.14],
                    "circular_direction": 0,
                    },
            }
            self._final_goals = {
                2: {
                    "position": [-1.8, 2.1],
                    "orientation": [1.0, 0.0, 0.0, 0.0],
                    },
                4: {
                    "position": [-8.0, -2.1],
                    "orientation": [1.0, 0.0, 0.0, 0.0],
                    },
                5: {
                    "position": [-6.0, -9.0],
                    "orientation": [1.0, 0.0, 0.0, 0.0],
                    },
                6: {
                    "position": [8.5, 8.5],
                    "orientation": [1.0, 0.0, 0.0, 0.0],
                    },
            }

        # Try to load the file
        try:
            with open(goals_path, "r") as file:
                goals = yaml.safe_load(file)
                if not goals:
                    raise ValueError("Empty or invalid YAML file")
                # extract the two main goal dictionaries from the goal dictionary
                self._cube2_goals = goals["cube2_goals"]
                self._final_goals = goals["final_goals"]
                self.get_logger().info(f"Loaded cube 2 goals from {goals_path}")
                
                # Display all the goal options to confirm
                for key, value in self._cube2_goals.items():
                    self.get_logger().info(
                        f"ArUco ID:{key} -- (x: {value['position'][0]}, {value['position'][1]})"
                    )
                self.get_logger().info(f"Loaded final goals from {goals_path}")
                # Display all the goal options to confirm
                for key, value in self._final_goals.items():
                    self.get_logger().info(
                        f"ArUco ID:{key} -- (x: {value['position'][0]}, {value['position'][1]})"
                    )
                    
                self.get_logger().info("All goals loaded successfully")

        except Exception as e:
            self.get_logger().error(f"Error loading goals file: {e}")
            # Return empty dict - service will return failure
            return  
# 

    def _get_cube2_goal_request(self, request: GetGoal.Request, response: GetGoal.Response)-> GetGoal.Response:
        """
        Handles GetGoal service request for cube #2 goals.

        :param request: GetGoal request, the aruco marker ID
        :type request: GetGoal.Request
        :param response: _description_
        :type response: GetGoal.Response
        :return: the goal pose, success boolean, and a message
        :rtype: GetGoal.Response
        """
        # Extract the goal parameters from the yaml dicionaries
        if request.id in self._cube2_goals.keys():
            goal = self._cube2_goals[request.id]
            
            # Fill out the service response
            response.position = goal['position']
            response.orientation = goal['orientation']
            response.circular_direction = goal['circular_direction']
            response.success = True
            response.message = f"Goal for ArUco id '{request.id}' provided successfully"

            self.get_logger().info(
                f"Provided goal for ArUco id '{request.id}' at position ({response.position[0]}, {response.position[1]})"
            )
        else:
            # If not present, fill out the appropriate response
            response.success = False
            response.message = f"No goal defined for ArUco id '{request.id}'"
            self.get_logger().warn(f"No goal defined for ArUco id '{request.id}'")

        return response
    

    def _get_final_goal_request(self, request: GetGoal.Request, response: GetGoal.Response) -> GetGoal.Response:
        """
        Handles GetGoal service request for final goals.

        :param request: GetGoal request, the aruco marker ID
        :type request: GetGoal.Request
        :param response: _description_
        :type response: GetGoal.Response
        :return: the goal pose, success boolean, and a message
        :rtype: GetGoal.Response
        """
        # Extract the goal parameters from the yaml dicionaries
        if request.id in self._final_goals.keys():
            goal = self._final_goals[request.id]
            # Fill out the service response
            response.position = goal['position']
            response.orientation= goal['orientation']
            response.circular_direction = 0 # UNUSED, DOES NOT MATTER
            response.success = True
            response.message = f"Goal for ArUco id '{request.id}' provided successfully"

            self.get_logger().info(
                f"Provided goal for ArUco id '{request.id}'  at position ({response.position[0]}, {response.position[1]})"
            )
        else:
            # If not present, fill out the appropriate response
            response.success = False
            response.message = f"No goal defined for ArUco id '{request.id}'"
            self.get_logger().warn(f"No goal defined for ArUco id '{request.id}'")

        return response


def main(args=None):
    rclpy.init(args=args)
    goal_provider = GoalProviderService()
    try:
        rclpy.spin(goal_provider)
    except KeyboardInterrupt:
        pass
    finally:
        goal_provider.destroy_node()
        rclpy.shutdown()
