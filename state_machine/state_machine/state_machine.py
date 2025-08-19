import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from ral_bunker_msgs.action import NextPose

class StateMachine(Node):

    def __init__(self):
        super().__init__('state_machine')
        self._bunker_action_client = ActionClient(self, NextPose, 'next_pose')

    def send_bunker_next_goal(self):
        goal_msg = NextPose.Goal()
        goal_msg.go_to_next_pose = True

        self._bunker_action_client.wait_for_server()

        # ASYNC VERSION
        self._send_bunker_goal_future = self._bunker_action_client.send_goal_async(goal_msg)
        self._send_bunker_goal_future.add_done_callback(self.bunker_goal_response_callback)

    def bunker_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')

        self._get_bunker_result_future = goal_handle.get_result_async()
        self._get_bunker_result_future.add_done_callback(self.get_bunker_result_callback)

    def get_bunker_result_callback(self, future):
        """
        This function is called when the bunker has finished
        """
        self.current_index = 0
        result = future.result().result
        self.get_logger().info(f'Result: {result}')


def main(args=None):
    rclpy.init(args=args)

    state_machine = StateMachine()

    executor = MultiThreadedExecutor()
    executor.add_node(state_machine)

    try:
        executor.spin()
    except KeyboardInterrupt:
        state_machine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()