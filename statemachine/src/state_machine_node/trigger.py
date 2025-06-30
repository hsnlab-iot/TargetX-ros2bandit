from __future__ import annotations  # allows forward references without quotes
from typing import Any, Dict, Type, Union
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from rclpy.duration import Duration
import threading
import time
import zmq
import zmq.utils.monitor as zmq_monitor
import json
import os
from dataclasses import dataclass, field
import rclpy

from area import Area
from connection import get_connection_handler

class TriggerManager:
    def __init__(self, node, parent):
        self._node = node
        self._parent = parent
        self.triggers = []
        self.areas = {}
        self.topics = {}
        self.actions = {}
        self.tf_buffer = None
        self.tf_listener = None

        self.trigger_classes: Dict[str, Type[Trigger]] = {
            "location_trigger": LocationTrigger,
            "topic_trigger": TopicTrigger,
            "time_trigger": TimeTrigger,
            "action_trigger": ActionTrigger,
        }

    # Recursive parser: replaces strings with Trigger objects
    def parse_triggers(self, expr: Any, state) -> tuple[Union[LogicalOperator, Trigger], list[Trigger]]:
        trigger_list = []

        def parse_triggers_r(expr: Any, state) -> Union[LogicalOperator, Trigger]:
            self._node.get_logger().debug(f"Parsing expression: {expr}")

            # Unwrap single-item list
            if isinstance(expr, list) and len(expr) == 1:
                expr = expr[0]

            if isinstance(expr, dict):
                for key, value in expr.items():
                    if key in {"and", "or"}:
                        children = []
                        for item in value:
                            child_node = parse_triggers_r(item, state)
                            children.append(child_node)
                        return LogicalOperator(key, children)

                    elif key == "not":
                        child_node = parse_triggers_r(value, state)
                        return LogicalOperator("not", [child_node])

                    elif key == "time_trigger":
                        trigger = TimeTrigger(id=expr["id"], state=state, params=value or {})
                        trigger_list.append(trigger)
                        return trigger

                    elif key == "topic_trigger":
                        trigger = TopicTrigger(id=expr["id"], state=state, topic=value)
                        trigger_list.append(trigger)
                        if value not in self.topics:
                            self.topics[value] = None
                        return trigger

                    elif key == "action_trigger":
                        trigger = ActionTrigger(id=expr["id"], state=state, action=value)
                        trigger_list.append(trigger)
                        if value not in self.actions:
                            self.actions[value] = None
                        return trigger

                    elif key == "location_trigger":
                        trigger = LocationTrigger(id=expr["id"], state=state, location=value)
                        trigger_list.append(trigger)
                        if value not in self.areas:
                            self.areas[value] = Area(value)
                        return trigger

                raise ValueError(f"Unsupported trigger format: {expr}")

        logic_tree = parse_triggers_r(expr, state)
        self.triggers.extend(trigger_list)
        self._node.get_logger().debug(f"Parsed logic tree: {logic_tree}")
        return logic_tree, trigger_list

    def evaluate_triggers(self, node: Union[Trigger, LogicalOperator]) -> bool:
        if isinstance(node, Trigger):
            return node.active

        elif isinstance(node, LogicalOperator):
            if node.op == "and":
                return all(self.evaluate_triggers(child) for child in node.children)
            elif node.op == "or":
                return any(self.evaluate_triggers(child) for child in node.children)
            elif node.op == "not":
                return not self.evaluate_triggers(node.children[0])
            else:
                raise ValueError(f"Unsupported logical operator: {node.op}")

        raise TypeError(f"Invalid node type: {type(node)}")
    
    def get_area_ids(self):
        return list(self.areas.keys())
        
    def load_areas(self, areas_section):
        if not self.areas:
            self._node.get_logger().debug("No areas to load.")
            return

        # Use ROS2 node to get a tf buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self._node)

        for area_id, area in self.areas.items():
            yaml_area = next((area for area in areas_section.values() if area.get('id') == area_id), None)
            if yaml_area:
                self._node.get_logger().debug(f"Loading area {area_id} from YAML section. {yaml_area}")
                area.setup(yaml_area)
            else:
                self._node.get_logger().warning(f"Area {area_id} not found in YAML section.")

    def watch_topics(self):

        self.watch_topics = get_connection_handler(self._node)
        self._node.get_logger().info(f"Watching topics: {self.topics}")

        def send_watch_topics():
            for topic in self.topics:
                message = {"topic": topic, "type": "watch"}
                self.watch_topics.send(message)
                self._node.get_logger().info(f"Sent watch for topic {topic}")

        send_watch_topics()
        self.watch_topics.register_onconnect(send_watch_topics)

        def on_message(message):
            if message.get("type") == "trigger":
                topic = message.get("topic")
                if topic is None:
                    return
                if topic in self.topics:
                    self.topics[topic] = time.time()
                    self._node.get_logger().debug(f"Updated topic {topic} with current time.")
                else:
                    self._node.get_logger().debug(f"Topic {topic} not found in topics list {self.topics}.")

        self.watch_topics.register_onmessage(on_message)

    def watch_actions(self):

        self.watch_actions = get_connection_handler(self._node)
        self._node.get_logger().info(f"Watching actions: {self.actions}")

        def send_watch_actions():
            for action in self.actions:
                message = {"action": action, "type": "watch"}
                self.watch_actions.send(message)
                self._node.get_logger().info(f"Sent watch for action {action}")

        send_watch_actions()
        self.watch_actions.register_onconnect(send_watch_actions)

        def on_message(message):
            if message.get("type") == "trigger":
                action = message.get("action")
                if action is None:
                    return
                if action in self.actions:
                    self.actions[action] = time.time()
                    self._node.get_logger().debug(f"Updated action {action} with current time.")
                else:
                    self._node.get_logger().debug(f"Action {action} not found in actions list {self.actions}.")

        self.watch_actions.register_onmessage(on_message)

    def update_triggers(self):
        self._node.get_logger().debug(f"Updating triggers: {self.triggers}")
        for trigger in self.triggers:
            trigger.update(self)

    def __repr__(self):
        return f"TriggerManager(triggers={[str(trigger) for trigger in self.triggers]})"

# Base trigger class
@dataclass
class Trigger:
    id: str
    state: Any
    active: bool = field(default=False, init=False)
    broker_connection: Any = field(default=None, init=False)

    def activate(self):
        if self.active:
            return
        self.active = True
        self.report_state()

    def deactivate(self):
        if not self.active:
            return
        self.active = False
        self.report_state()

    def is_active(self) -> bool:
        return self.active

    def update(self, context):
        pass
        #raise NotImplementedError("Subclasses should implement this method.")

    def report_state(self):
        if self.broker_connection is None:
            self.broker_connection = get_connection_handler(self.state._parent._node)

        message = {
            "type": "trigger_change",
            "trigger": self.id,
            "state": self.state.id,
            "state_machine": self.state._parent.id,
            "active": self.active
        }
        self.broker_connection.send(message)

@dataclass
class LocationTrigger(Trigger):
    location: str

    def update(self, context):
        area = context.areas.get(self.location)
        if not area:
            return
        transform: TransformStamped = None
        try:
            transform = context.tf_buffer.lookup_transform(
                area.ref, area.tf, rclpy.time.Time(),
                timeout = rclpy.duration.Duration(seconds=0.01))
        except TransformException as e:
            context._node.get_logger().debug(f"ref: {area.ref}, tf: {area.tf}")
            context._node.get_logger().warning(f"Transform error: {e}")
            return
        
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        active = area.inside([x, y])
        if active != self.is_active():
            if active:
                self.activate()
                context._node.get_logger().debug(f"LocationTrigger {self.location} activated at {x}, {y} in area {area}.")
            else:
                self.deactivate()
                context._node.get_logger().debug(f"LocationTrigger {self.location} deactivated at {x}, {y} in area {area}.")

    def __repr__(self):
        return f"LocationTrigger(location={self.location}, active={self.active})"

@dataclass
class TopicTrigger(Trigger):
    topic: str

    def update(self, context):
        active = False
        if self.topic in context.topics.keys():
            topic_time = context.topics[self.topic]
            if topic_time is not None:
                if time.time() - topic_time < 0.5:
                    active = True

        if active != self.is_active():
            if active:
                self.activate()
                context._node.get_logger().info(f"TopicTrigger {self} activated.")
            else:
                self.deactivate()
                context._node.get_logger().info(f"TopicTrigger {self} deactivated.")


    def __repr__(self):
        return f"TopicTrigger(topic={self.topic}, active={self.active})"

@dataclass
class ActionTrigger(Trigger):
    action: str

    def update(self, context):
        active = False
        if self.action in context.actions.keys():
            action_time = context.actions[self.action]
            if action_time is not None:
                if time.time() - action_time < 0.5:
                    active = True

        if active != self.is_active():
            if active:
                self.activate()
                context._node.get_logger().info(f"ActionTrigger {self} activated.")
            else:
                self.deactivate()
                context._node.get_logger().info(f"ActionTrigger {self} deactivated.")


    def __repr__(self):
        return f"ActionTrigger(action={self.action}, active={self.active})"

@dataclass
class TimeTrigger(Trigger):
    params: dict = field(default_factory=dict)

    def __post_init__(self):
        params = self.params  # local shortcut

        self.last_change = Duration(seconds=float(params["last_change"])) if "last_change" in params else None
        self.in_state = Duration(seconds=float(params["in_state"])) if "in_state" in params else None

    def update(self, context):
        now  = context._node.get_clock().now()

        active = False
        if self.last_change is not None and self.state._parent.last_change is not None:
            # Check if the last change is within the time limit

            active = self.last_change < now - self.state._parent.last_change

        if not self.in_state is None:
            active = self.state._parent.active_state_id == self.state.id and self.in_state > now - self.state._parent.last_change

        if active != self.is_active():
            if active:
                self.activate()
                context._node.get_logger().debug(f"TimeTrigger {self} activated in state {self.state._parent.active_state_id}.")
            else:
                self.deactivate()
                context._node.get_logger().debug(f"TimeTrigger {self} deactivated in state {self.state._parent.active_state_id}.")

    def __repr__(self):
        return f"TimeTrigger(last_change={self.last_change}, in_state={self.in_state}, active={self.active})"
    
@dataclass
class LogicalOperator:
    op: str  # "and", "or", "not"
    children: list
