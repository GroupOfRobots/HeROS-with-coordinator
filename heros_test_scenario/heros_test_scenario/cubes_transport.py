import time
import rclpy
from simple_node import Node
from yasmin import State
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray

# Conditions of state transitions
CUBE_PLACED_ON_MINIRYS = False
CUBE_TAKEN_OFF_MINIRYS = False
NO_MORE_CUBES_LEFT = False
GOAL_SUCCEEDED = False

class PlaceCubeOnRys(State, Node):
    def __init__(self):
        super().__init__(["done", "failed", "no cubes"])
        Node.__init__(self, "yasmin_node_place_cube") 
        self.publisher_ = self.create_publisher(String, '/heros_tasks', 10)

    def execute(self, blackboard):
        self.get_logger().info("Executing state PlaceCubeOnRys")
        self.publisher_.publish(String(data = 'load'))
        while (not CUBE_PLACED_ON_MINIRYS) and (not NO_MORE_CUBES_LEFT):
            self.get_logger().info('Waiting for cube to be loaded')
            time.sleep(1)

        if CUBE_PLACED_ON_MINIRYS == True:
             return "done"
        elif NO_MORE_CUBES_LEFT == True:
            return "no cubes"

class DriveToTarget(State, Node):
    def __init__(self):
        super().__init__(["done", "failed"])
        Node.__init__(self, "yasmin_node_drive_to_target") 
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        global GOAL_SUCCEEDED

    def execute(self, blackboard):
        self.get_logger().info("Executing state DriveToTarget") # TODO Kuba
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = 0.9884449211724036
        pose_msg.pose.position.y = -0.0538024593238705
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.7092453993500845
        pose_msg.pose.orientation.w = 0.7049616751999638
        self.publisher_.publish(pose_msg)
        while not GOAL_SUCCEEDED:
            self.get_logger().info('Waiting for robot to drive to target')
            self.get_logger().info('Bool "%s"' % GOAL_SUCCEEDED)
            time.sleep(1) 
        return "done"
       
    
class PickUpFromRys(State, Node):
    def __init__(self):
        super().__init__(["done", "failed"])
        Node.__init__(self, "yasmin_node_pick_up_cube") 
        self.publisher_ = self.create_publisher(String, '/heros_tasks', 10)

    def execute(self, blackboard):
        self.get_logger().info("Executing state PickUpFromRys")
        self.publisher_.publish(String(data = 'unload'))
        while not CUBE_TAKEN_OFF_MINIRYS:
            self.get_logger().info('Waiting for cube to be unloaded')
            time.sleep(1)
        return "done"
    
    
class DriveToSupply(State, Node):
    def __init__(self):
        super().__init__(["done", "failed"])
        Node.__init__(self, "yasmin_node_drive_to_supply") 
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def execute(self, blackboard):
        self.get_logger().info("Executing state DriveToSupply") # TODO Kuba
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = 0.05452109729691368
        pose_msg.pose.position.y = -0.4812417139251351
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = -0.6766263114289787
        pose_msg.pose.orientation.w = 0.7363265815397504
        self.publisher_.publish(pose_msg)
        while not GOAL_SUCCEEDED:
            self.get_logger().info('Waiting for robot to drive to supply')
            time.sleep(1) 
        return "done"


class TestScenarioNode(Node):

    def __init__(self):
        super().__init__("yasmin_node")
        global GOAL_SUCCEEDED
        # self.goal_succeed = False

        # Subscribers
        self.loading_subscription = self.create_subscription(
            String,
            '/loading_manipulator',
            self.loading_manipulator_callback,
            10)

        self.unloading_subscription = self.create_subscription(
            String,
            '/unloading_manipulator',
            self.unloading_manipulator_callback,
            10)
        
        self.goal_status_subscription = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.pose_status_callback,
            10)
        
        # create a state machine
        sm = StateMachine(outcomes=["Task accomplished", "Abort task execution", "Path is blocked"])

        # add states
        sm.add_state("[Manipulator] Place cube on mobile robot", PlaceCubeOnRys(),
                     transitions={"done": "[Mobile robot] Drive to unloading zone",
                                  "failed": "Abort task execution",
                                  "no cubes": "Task accomplished"})
        sm.add_state("[Mobile robot] Drive to unloading zone", DriveToTarget(),
                     transitions={"done": "[Manipulator] Pick up cube from mobile robot",
                                  "failed": "Path is blocked"})
        sm.add_state("[Manipulator] Pick up cube from mobile robot", PickUpFromRys(),
                     transitions={"done": "[Mobile robot] Drive to loading zone",
                                  "failed": "Abort task execution"})
        sm.add_state("[Mobile robot] Drive to loading zone", DriveToSupply(),
                     transitions={"done": "[Manipulator] Place cube on mobile robot",
                                  "failed": "Path is blocked"})
        
        YasminViewerPub(self, "HEROS_SCENARIO", sm)

        # execute
        outcome = sm()
        print(outcome)

    # def getGoalFlag(self):
    #     return self.goal_succeed

    # def resetGoalFlag(self):
    #     self.goal_succeed = False

    def loading_manipulator_callback(self, msg):
        self.get_logger().info('FSM received message: "%s"' % msg.data)
        global GOAL_SUCCEEDED
        if msg.data == "loaded":
            global CUBE_PLACED_ON_MINIRYS
            CUBE_PLACED_ON_MINIRYS = True

            global CUBE_TAKEN_OFF_MINIRYS
            CUBE_TAKEN_OFF_MINIRYS = False

            GOAL_SUCCEEDED = False
        elif msg.data == "empty":
            global NO_MORE_CUBES_LEFT
            NO_MORE_CUBES_LEFT = True

    def unloading_manipulator_callback(self, msg):
        global GOAL_SUCCEEDED
        self.get_logger().info('FSM received message: "%s"' % msg.data)
        if msg.data == "unloaded":
            global CUBE_TAKEN_OFF_MINIRYS
            CUBE_TAKEN_OFF_MINIRYS = True

            global CUBE_PLACED_ON_MINIRYS
            CUBE_PLACED_ON_MINIRYS = False

            GOAL_SUCCEEDED = False

    def pose_status_callback(self, msg):
        self.get_logger().info('FSM received GoalStatus message: "%s"' % msg.status_list[0].status)
        global GOAL_SUCCEEDED
        if msg.status_list[0].status == 4:
            self.get_logger().info('IM IN')
            # goal_succeed = True
            # global GOAL_SUCCEEDED
            GOAL_SUCCEEDED = True
        else:
            self.get_logger().info('IM OUT')
            # goal_succeed = False
            # global GOAL_SUCCEEDED
            GOAL_SUCCEEDED = False

def main(args=None):

    rclpy.init(args=args)
    node = TestScenarioNode()
    node.join_spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    