from trigger import TriggerManager
from connection import get_connection_handler

class State:
    def __init__(self, id, parent):
        self.id = id
        self._parent = parent
        self.priority = 9999
        self.blocked_topics = {}
        
        self.trigger_logic = None

    def setup(self, yaml_section):
        self.id = yaml_section.get('id', self.id)
        self.priority  = yaml_section.get('priority', None)
        self.blocked_topics = yaml_section.get('blocked_topics', [])
        if self.blocked_topics is None:
            self.blocked_topics = []

        # Trigger and Logic Tree
        trigger = yaml_section.get('trigger', None)
        if trigger:
            self.trigger_logic, self.triggers = self._parent.trigger_mngr.parse_triggers(trigger, self)

    def __repr__(self):
        return f"State(id={self.id}, priority={self.priority}, blocked_topics={self.blocked_topics}, trigger_logic={self.trigger_logic})"

class StateMachine:
    def __init__(self, id, yaml_section, node):
        self.id = id
        self._node = node
        self.trigger_mngr = TriggerManager(self._node, self)

        self.active_state_id = None
        self.last_change = self._node.get_clock().now()

        self.states = {state_id: State(state_id, self) for state_id in yaml_section.get('states', [])}
        self.areas = []

        self.broker_connection = get_connection_handler(self._node)

    def load_states(self, states_section, areas_section):
        for state_id, state in self.states.items():
            yaml_state = next((state for state in states_section.values() if state.get('id') == state_id), None)
            if yaml_state:
                self._node.get_logger().debug(f"Loading state {state_id} from YAML section. {yaml_state}")
                state.setup(yaml_state)
            else:
                self._node.get_logger().warning(f"State {state_id} not found in YAML section.")

        self.areas = self.trigger_mngr.get_area_ids()
        self.trigger_mngr.load_areas(areas_section)
        
        self.trigger_mngr.watch_topics()
        self.trigger_mngr.watch_actions()
        
        # Sort states by priority
        self.states = dict(sorted(self.states.items(), key=lambda item: item[1].priority))

        # Send states to broker
        self.send_states()

    def send_states(self):
        states = {}
        for state_id, state in self.states.items():
            triggers = []
            for trigger in state.triggers:
                triggers.append({'id': trigger.id, 'type': trigger.short_type()})
            states[state.id] = {'priority': state.priority,
                        'blocked_topics': state.blocked_topics,
                        'triggers': triggers}
        self.broker_connection.send({'type': 'statemachines_description', 'state_machine': self.id, 'states': states})
        self._node.get_logger().debug(f"StateMachine(id={self.id}) sending states: {states}")

        self.broker_connection.register_onconnect(self.send_states)

    def update(self):
        self.trigger_mngr.update_triggers()

        for state_id, state in self.states.items():
            if state.trigger_logic:
                active = self.trigger_mngr.evaluate_triggers(state.trigger_logic)
                #if active:
                #    self._node.get_logger().info(f"StateMachine(id={self.id}, state {state_id} is active)")
                #    self._node.get_logger().info(f"state.trigger_logic: {state.trigger_logic}")
                if active:
                    if self.active_state_id != state_id:
                        self._node.get_logger().info(f"StateMachine(id={self.id}, state changed from {self.active_state_id} to {state_id}")
                        self._node.get_logger().info(f"state.trigger_logic: {state.trigger_logic}")

                        # Update blocked topics
                        oldstate = self.states.get(self.active_state_id)
                        newstate = self.states.get(state_id)
                        if oldstate and oldstate.blocked_topics:
                            self._node.get_logger().info(f"oldstate.blocked_topics: {oldstate.blocked_topics}")
                            for topic in oldstate.blocked_topics:
                                if (not newstate.blocked_topics) or (topic not in newstate.blocked_topics):
                                    message = {'topic': topic, 'type': 'unblock'}
                                    self.broker_connection.send(message)

                        if newstate.blocked_topics:                                
                            self._node.get_logger().info(f"newstate.blocked_topics: {newstate.blocked_topics}")
                            for topic in newstate.blocked_topics:
                                if not oldstate or (not oldstate.blocked_topics) or (topic not in oldstate.blocked_topics):
                                    message = {'topic': topic, 'type': 'block'}
                                    self.broker_connection.send(message)

                        broker_message = {
                            'type': 'state_change',
                            'state_machine': self.id,
                            'old_state': self.active_state_id,
                            'new_state': state_id
                        }
                        self.broker_connection.send(broker_message)

                        self.active_state_id = state_id
                        self.last_change = self._node.get_clock().now()
                    break

    def __repr__(self):
        return f"StateMachine(id={self.id}, states={', '.join(str(state) for state in self.states.values())}, areas={self.areas})"
