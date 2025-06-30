from flask import Flask, Response, jsonify, request
from flask_cors import CORS
import time
import random
from shared_memory_dict import SharedMemoryDict
import pickle
import json
from graphviz import Digraph


def create_app(shared, lock):
    app = Flask(__name__)
    CORS(app)
    print("App created:", type(app))

    def _get_topic_traffic():
        shared_topic_traffic = SharedMemoryDict(name="topic_traffic", size=10240)
        with lock:
            topic_traffic = {}
            for key in shared_topic_traffic.keys():
                topic_traffic[key] = pickle.loads(shared_topic_traffic[key])
                if topic_traffic[key]['update'] < time.time() - 3:
                    topic_traffic[key]['bw'] = 0
                    topic_traffic[key]['hz'] = 0
            print("Topic traffic data:", topic_traffic)
        return topic_traffic

    @app.route('/get-topic-traffic')
    def get_topic_traffic():
        return jsonify(_get_topic_traffic())

    @app.route('/stream-topic-traffic')
    def stream_topic_traffic():
        def event_stream():
            while True:
                topic_traffic = _get_topic_traffic()
                yield f"data: {json.dumps(topic_traffic)}\n\n"
                time.sleep(0.1)

        return Response(event_stream(), mimetype='text/event-stream')

    @app.route('/get-statemachines')
    def get_states():
        shared_states = SharedMemoryDict(name="statemachines_description", size=10240)
        with lock:
            states = {}
            for key in shared_states.keys():
                states[key] = pickle.loads(shared_states[key])
            print("Topic traffic data:", states)
        return jsonify(states)

    @app.route('/get-statemachines-diagram')
    def get_state_machine_diagram():
        shared_states = SharedMemoryDict(name="statemachines_description", size=10240)
        svgs = {}
        with lock:
            for sm_name, states in shared_states.items():
                states = pickle.loads(states)
                dot = Digraph(format='svg')
                dot.attr(rankdir='LR', bgcolor='white')
                dot.attr('node', shape='ellipse', style='filled', fillcolor='lightgray', fontname='Helvetica', fontcolor='black')
                with dot.subgraph(name=f"cluster_{sm_name}") as sm_graph:
                    sm_graph.attr(label=sm_name, style='dashed', color='gray')
                    for state_name, state_data in states.items():
                        node_id = f"{sm_name}/{state_name}"
                        label = f"{state_name}\n(priority: {state_data['priority']})"
                        sm_graph.node(node_id, label=label, id=node_id)
                    for state_name, state_data in states.items():
                        for trigger in state_data.get("triggers", []):
                            trigger_id = trigger["id"]
                            trigger_type = trigger["type"]
                            trigger_id_full = f"{sm_name}/{state_name}/{trigger_id}"
                            trigger_label = f"{trigger_type}| {trigger_id}"
                            state_node = f"{sm_name}/{state_name}"
                            dot.node(trigger_id_full, label=trigger_label, shape='box', style='rounded,filled', id=trigger_id_full)
                            dot.edge(trigger_id_full, state_node)
                svg_bytes = dot.pipe(format='svg')
                svgs[sm_name] = svg_bytes.decode('utf-8')
        return jsonify(svgs)

    @app.route('/stream-states')
    def stream_states():
        shared_states = SharedMemoryDict(name="states", size=10240)
        shared_triggers = SharedMemoryDict(name="triggers", size=10240)

        last_states = {}
        last_triggers = {}

        def event_stream():
            while True:
                changed_states = {}
                changed_triggers = {}
                with lock:
                    for s in shared_states.keys():
                        if s not in last_states or last_states[s] != shared_states[s]:
                            changed_states[s] = shared_states[s]
                            last_states[s] = shared_states[s]
                    for t in shared_triggers.keys():
                        if t not in last_triggers or last_triggers[t] != shared_triggers[t]:
                            changed_triggers[t] = shared_triggers[t]
                            last_triggers[t] = shared_triggers[t]

                if changed_states or changed_triggers:
                    payload = {
                        "data-states": changed_states,
                        "data-triggers": changed_triggers,
                    }
                    yield f"data: {json.dumps(payload)}\n\n"
                
                time.sleep(0.1)
        
        return Response(event_stream(), mimetype='text/event-stream')

    @app.route('/get-actions')
    def get_actions():
        shared_actions = SharedMemoryDict(name="actions", size=10240)
        with lock:
            actions = {}
            for key in shared_actions.keys():
                actions[key] = shared_actions[key]
            print("Actions data:", actions)
        return jsonify(actions)

    @app.route('/data/chart')
    def get_chart_data():
        menu = request.args.get('menu', 'Topics')
        # Sample data (replace with real logic)
        return jsonify({
            'menu': menu,
            'value': [1, 2, 3, 4, 5],
        })

    @app.route('/stream')
    def stream():
        def event_stream():
            while True:
                value = random.randint(0, 100)
                yield f"data: {value}\n\n"
                time.sleep(0.1)

        return Response(event_stream(), mimetype='text/event-stream')

    return app
