import rclpy
from rclpy.node import Node
import yaml
import threading
import time
from pathlib import Path
import os

from state_machine import StateMachine
from rclpy.logging import LoggingSeverity

stms = {}

class StateMachineNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info('State Machine Node has been started.')

    def reload(self, state_machine_config):
        self.get_logger().info('Reloading state machine configuration.')

        try:
            with open(state_machine_config, 'r') as file:
                config = yaml.safe_load(file)
                global stms
                
                stms = config.get('state_machines', {})
                stms = {stm['id']: StateMachine(stm['id'], stm, self) for stm in config.get('state_machines', [])}

                states = config.get('states', {})
                states = {state['id']: state for state in config.get('states', [])}

                areas = config.get('areas', {})
                areas = {area['id']: area for area in config.get('areas', [])}

                # Load system configuration
                system_config = config.get('system', {})
                self.rate_hz = system_config.get('rate_hz', None)
                self.location_check_m = system_config.get('location_check_m', None)

                for stm_id, stm in stms.items():
                    stm.load_states(states, areas)

                self.get_logger().info(f'Successfully loaded configuration.')
                
        except FileNotFoundError:
            self.get_logger().error(f'Configuration file {state_machine_config} not found.')
        except yaml.YAMLError as e:
            self.get_logger().error(f'Error parsing YAML file: {e}')

        # Start a thread to periodically update state machines
        if self.rate_hz:

            def update_state_machines():
                rate = 1.0 / self.rate_hz
                while rclpy.ok():
                    for stm in stms.values():
                        stm.update()
                    time.sleep(rate)

            self.update_thread = threading.Thread(target=update_state_machines, daemon=True)
            self.update_thread.start()

def main(args=None):
    rclpy.init(args=args)
    name = os.environ.get('STATE_MACHINE_NAME', 'state_machine_node')
    node = StateMachineNode(name)
    #node.get_logger().set_level(LoggingSeverity.DEBUG)

    file_path = Path('/config/statemachines.yaml')
    if not file_path.exists():
        node.get_logger().error(f'Configuration file {file_path} does not exist.')
        return

    node.reload(file_path.__str__())
    node.get_logger().info(f'State Machines: {stms}')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('State Machine Node is shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()