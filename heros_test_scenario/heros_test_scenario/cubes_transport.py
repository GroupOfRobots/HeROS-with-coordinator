import time
import rclpy
from simple_node import Node
from yasmin import State
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub
from std_msgs.msg import String

# Conditions of state transitions
CUBE_PLACED_ON_MINIRYS = False
CUBE_TAKEN_OFF_MINIRYS = False
NO_MORE_CUBES_LEFT = False


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

class DriveToTarget(State):
    def __init__(self):
        super().__init__(["done", "failed"])

    def execute(self, blackboard):
        print("Executing state DriveToTarget") # TODO Kuba
        time.sleep(5)       
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
    
class DriveToSupply(State):
    def __init__(self):
        super().__init__(["done", "failed"])

    def execute(self, blackboard):
        print("Executing state DriveToSupply") # TODO Kuba
        time.sleep(5)  
        return "done"


class TestScenarioNode(Node):

    def __init__(self):
        super().__init__("yasmin_node")

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

    def loading_manipulator_callback(self, msg):
        self.get_logger().info('FSM received message: "%s"' % msg.data)
        if msg.data == "loaded":
            global CUBE_PLACED_ON_MINIRYS
            CUBE_PLACED_ON_MINIRYS = True

            global CUBE_TAKEN_OFF_MINIRYS
            CUBE_TAKEN_OFF_MINIRYS = False
        elif msg.data == "empty":
            global NO_MORE_CUBES_LEFT
            NO_MORE_CUBES_LEFT = True

    def unloading_manipulator_callback(self, msg):
        self.get_logger().info('FSM received message: "%s"' % msg.data)
        if msg.data == "unloaded":
            global CUBE_TAKEN_OFF_MINIRYS
            CUBE_TAKEN_OFF_MINIRYS = True

            global CUBE_PLACED_ON_MINIRYS
            CUBE_PLACED_ON_MINIRYS = False

def main(args=None):

    rclpy.init(args=args)
    node = TestScenarioNode()
    node.join_spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()