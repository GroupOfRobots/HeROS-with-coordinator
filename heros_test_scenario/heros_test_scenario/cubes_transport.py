import time
import rclpy
from simple_node import Node
from yasmin import State
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub


class PlaceCubeOnRys(State):
    def __init__(self):
        super().__init__(["done", "failed", "no cubes"])

    def execute(self, blackboard):
        print("Executing state PlaceCubeOnRys") # TODO Janek
        time.sleep(5)
        return "done"

class DriveToTarget(State):
    def __init__(self):
        super().__init__(["done", "failed"])

    def execute(self, blackboard):
        print("Executing state DriveToTarget") # TODO Kuba
        time.sleep(5)       
        return "done"
    
class PickUpFromRys(State):
    def __init__(self):
        super().__init__(["done", "failed"])

    def execute(self, blackboard):
        print("Executing state PickUpFromRys") # TODO Janek
        time.sleep(5)  
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


def main(args=None):

    rclpy.init(args=args)
    node = TestScenarioNode()
    node.join_spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()