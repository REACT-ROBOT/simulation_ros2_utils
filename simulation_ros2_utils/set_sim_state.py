import os
import rclpy
from rclpy.node import Node
from simulation_interfaces.srv import SetSimulationState
from simulation_interfaces.msg import SimulationState

class SetSimStateNode(Node):
    def __init__(self):
        super().__init__('set_sim_state_node')

        request = SetSimulationState.Request()

        self.declare_parameter('set_state', '')
        set_state = self.get_parameter('set_state').get_parameter_value().string_value
        if set_state == '':
            return
        if set_state == 'stop':
            request.state.state = SimulationState.STATE_STOPPED
        elif set_state == 'start':
            request.state.state = SimulationState.STATE_PLAYING
        elif set_state == 'pause':
            request.state.state = SimulationState.STATE_PAUSED
        elif set_state == 'quit':
            request.state.state = SimulationState.STATE_QUITTING
        else:
            self.get_logger().error("Invalid state. Use 'stop', 'start', 'pause', or 'quit'.")
            return

        # SetSimulationStateサービスクライアントを作成
        self.simulation_state_client = self.create_client(SetSimulationState, 'set_simulation_state')
        while not self.simulation_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_simulation_state service not available, waiting...')
        
        self.get_logger().info("Sending state request...")
        
        # サービス呼び出し
        future = self.simulation_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.result.result == 0:  # SUCCESS
                self.get_logger().info(f"Simulation state set to {set_state}")
            else:
                self.get_logger().error(f"Failed to set simulation state: {response.result.message}")
        else:
            self.get_logger().error("Exiting node due to service call failure")

def main(args=None):
    rclpy.init(args=args)
    
    node = SetSimStateNode()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

