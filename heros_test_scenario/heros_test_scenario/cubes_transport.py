import time
import rclpy
from simple_node import Node
from yasmin import StateMachine, Blackboard
from yasmin_ros import MonitorState
from yasmin_viewer import YasminViewerPub
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped


class PlaceCubeOnRys(MonitorState):
    def __init__(self, node: Node) -> None:
        super().__init__(node,  String,  "/loading_manipulator", ["done", "failed", "no cubes"], 
                        self.state_callback, qos=10, msg_queue=10, timeout=None)
        self.node_ = node
        self.publisher_dobot = node.create_publisher(String, '/heros_tasks', 10)
        print("Starting finite state machine in 5 seconds ...")
        time.sleep(5)
        print("Executing state PlaceCubeOnRys")
        self.publisher_dobot.publish(String(data = 'load'))
        self.publisher_rys = node.create_publisher(PoseStamped, '/goal_pose', 10)
        
    def state_callback(self, blackboard: Blackboard) -> str:
        self.node_.get_logger().info("Received msg: " + str(blackboard.msg))
        if blackboard.msg.data == "loaded":
            print("Executing state DriveToTarget")
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.node_.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = 0.9884449211724036
            pose_msg.pose.position.y = -0.0538024593238705
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.7092453993500845
            pose_msg.pose.orientation.w = 0.7049616751999638
            self.publisher_rys.publish(pose_msg)
            return "done"
        elif blackboard.msg.data == "empty":
            return "no cubes"

class DriveToTarget(MonitorState):
    def __init__(self, node: Node) -> None:
        super().__init__(node,  GoalStatusArray,  "/navigate_to_pose/_action/status", ["done", "failed"],
                        self.state_callback, qos=10, msg_queue=10, timeout=None)
        self.node_ = node
        self.publisher_dobot = node.create_publisher(String, '/heros_tasks', 10)

    def state_callback(self, blackboard: Blackboard) -> str:
        self.node_.get_logger().info("Received msg: " + str(blackboard.msg))
        if blackboard.msg.status_list[0].status == 4:
            print("Executing state PickUpFromRys")
            self.publisher_dobot.publish(String(data = 'unload'))
            return "done"
    
class PickUpFromRys(MonitorState):
    def __init__(self, node: Node) -> None:
        super().__init__(node,  String,  "/unloading_manipulator", ["done", "failed"], 
                        self.state_callback, qos=10, msg_queue=10, timeout=None)
        self.node_ = node
        self.publisher_rys = node.create_publisher(PoseStamped, '/goal_pose', 10)

    def state_callback(self, blackboard: Blackboard) -> str:
        self.node_.get_logger().info("Received msg: " + str(blackboard.msg))
        if blackboard.msg.data == "unloaded":
            print("Executing state DriveToSupply")
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.node_.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = 0.05452109729691368
            pose_msg.pose.position.y = -0.4812417139251351
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = -0.6766263114289787
            pose_msg.pose.orientation.w = 0.7363265815397504
            self.publisher_rys.publish(pose_msg)
            return "done"
    
class DriveToSupply(MonitorState):
    def __init__(self, node: Node) -> None:
        super().__init__(node,  GoalStatusArray,  "/navigate_to_pose/_action/status", ["done", "failed"],
                        self.state_callback, qos=10, msg_queue=10, timeout=None)
        self.node_ = node
        self.publisher_dobot = node.create_publisher(String, '/heros_tasks', 10)

    def state_callback(self, blackboard: Blackboard) -> str:
        self.node_.get_logger().info("Received msg: " + str(blackboard.msg))
        if blackboard.msg.status_list[0].status == 4:
            print("Executing state PlaceCubeOnRys")
            self.publisher_dobot.publish(String(data = 'load'))
            return "done"


class TestScenarioNode(Node):

    def __init__(self):
        super().__init__("yasmin_node")

        # create a state machine
        sm = StateMachine(outcomes=["Task accomplished", "Abort task execution", "Path is blocked"])

        # add states
        sm.add_state("[Manipulator] Place cube on mobile robot", PlaceCubeOnRys(self),
                     transitions={"done": "[Mobile robot] Drive to unloading zone",
                                  "failed": "Abort task execution",
                                  "no cubes": "Task accomplished"})
        sm.add_state("[Mobile robot] Drive to unloading zone", DriveToTarget(self),
                     transitions={"done": "[Manipulator] Pick up cube from mobile robot",
                                  "failed": "Path is blocked"})
        sm.add_state("[Manipulator] Pick up cube from mobile robot", PickUpFromRys(self),
                     transitions={"done": "[Mobile robot] Drive to loading zone",
                                  "failed": "Abort task execution"})
        sm.add_state("[Mobile robot] Drive to loading zone", DriveToSupply(self),
                     transitions={"done": "[Manipulator] Place cube on mobile robot",
                                  "failed": "Path is blocked"})
        
        YasminViewerPub(self, "HEROS_SCENARIO", sm)

        # execute
        outcome = sm()
        print(outcome)


def main(args=None):

    rclpy.init(args=args)
    node = TestScenarioNode()
    node.join_spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
