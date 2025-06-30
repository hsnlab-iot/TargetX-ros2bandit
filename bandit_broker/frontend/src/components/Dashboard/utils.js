import * as yaml from 'js-yaml';

export function formatBandwidth(bps) {
  if (bps >= 1e9) return (bps / 1e9).toFixed(2) + ' GBps';
  if (bps >= 1e6) return (bps / 1e6).toFixed(2) + ' MBps';
  if (bps >= 1e3) return (bps / 1e3).toFixed(2) + ' KBps';
  return bps + ' Bps';
}
  
export function updateShapeClass(id, type, active) {
  const el = document.getElementById(id);
  if (!el) return;

  const shape = el.querySelector('path, rect, polygon, ellipse, circle');
  if (!shape) return;

  if (type === 'state') {
    shape.classList.remove('state-active-box', 'state-inactive-box');
    shape.classList.add(active ? 'state-active-box' : 'state-inactive-box');
  } else if (type === 'trigger') {
    shape.classList.remove('trigger-active-box', 'trigger-inactive-box');
    shape.classList.add(active ? 'trigger-active-box' : 'trigger-inactive-box');
  }
}

// Helper function to calculate the label width based on text length
const getNodeWidth = (label) => {
  // Set a minimum and maximum width based on label length
  const minWidth = 100;
  const maxWidth = 300;
  const labelLength = label.length;

  // Dynamically calculate width based on the label length
  return Math.min(maxWidth, Math.max(minWidth, labelLength * 10)); // Scale by 10 for padding and extra space
};

// Function to convert the YAML description into React Flow elements
export const createElementsFromYAML = (description) => {
  console.log("Processing YAML data...");
  const nodes = [];
  const edges = [];
  let nodeId = 0;

  // Helper function to process triggers and operators
  const processTrigger = (trigger, parentNodeId, stateMachineNodeId) => {

    console.log("Processing trigger:", trigger, "Parent Node ID:", parentNodeId, "State Machine ID:", stateMachineNodeId);

    // Handle logic operators like 'or', 'and', or 'not'
    if (trigger.or || trigger.and || trigger.not) {
      const operatorType = trigger.or ? 'or' : trigger.and ? 'and' : 'not';
      const operatorNodeId = `operator-${nodeId++}`;      
      // Create operator node
      const operatorNode = addNodeStyle({
        id: operatorNodeId,
        operatorType: operatorType,
        position: { x: 0, y: 0 }, // Position will be set by Dagre
        parent: stateMachineNodeId,
      }, 'operator');
      nodes.push(operatorNode);
    
      // Check if the operator has triggers inside it and process them
      const nestedTriggers = trigger[operatorType] || [];
      if (nestedTriggers) {
        nestedTriggers.forEach((nestedTrigger) => {
          processTrigger(nestedTrigger, operatorNodeId, stateMachineNodeId);  // Recursively process nested triggers
        });
      }

      // Add edge from the parent node to the operator node
      edges.push({
        id: `${operatorNodeId}-${parentNodeId}`,
        source: operatorNodeId,
        target: parentNodeId,
        animated: true,
        style: { strokeWidth: 4 }, // <-- bolder edge
        interactive: true, // Allow interaction with the edge
      });
    } else {

      const triggerNodeId = `trigger-${nodeId++}`;
      // Create trigger node
      const triggerNode = 
        addNodeStyle({
        id: triggerNodeId,
        trigger: trigger, // Store the trigger data for later use
        position: { x: 0, y: 0 }, // Position will be set by Dagre
        parent: stateMachineNodeId,
      }, 'trigger');
      nodes.push(triggerNode);

      // Add edge between the parent state node and trigger node
      edges.push({
        id: `${triggerNodeId}-${parentNodeId}`,
        source: triggerNodeId,
        target: parentNodeId,
        animated: true,
        style: { strokeWidth: 4 }, // <-- bolder edge
        interactive: true, // Allow interaction with the edge
      });
    }

  };

  // Process state machines and their states
  description.state_machines.forEach((stateMachine) => {
    const stateMachineId = stateMachine.id;
    //const stateMachineNodeId = `state-machine-${nodeId++}`;
    const stateMachineNodeId = stateMachine.id;
    const stateMachinePosition = { x: 0, y: 0 }; // Initial position, will be updated by Dagre

    // Process states for each state machine
    stateMachine.states.forEach((stateName) => {
      const stateData = description.states.find((state) => state.id === stateName);
      if (!stateData) {
        console.warn(`State '${stateName}' not found in description.`);
        return;
      }

      const stateNodeId = `state-${nodeId++}`;
      // Create state node inside the group
      const stateNode = addNodeStyle({
        id: stateNodeId,
        stateData: stateData, // Store state data for later use
        position: { x: 0, y: 0 }, // Position will be set by Dagre
        parent: stateMachineNodeId,
        extent: 'parent',
      }, 'state');
      nodes.push(stateNode);

      // Process triggers for each state
      if (stateData.trigger) {
        if (Array.isArray(stateData.trigger)) {
          stateData.trigger.forEach((trigger) => {
            processTrigger(trigger, stateNodeId, stateMachineNodeId);  // Process each trigger recursively
          });
        } else {
          processTrigger(stateData.trigger, stateNodeId, stateMachineNodeId); // If not an array, process directly
        }
      }
    });
  });

  console.log("Created nodes and edges:", nodes, edges);
  return { nodes, edges };
};

export const addNodeStyle = (node, type) => {
  node.nodeType = type; // Store the type for later reference
  node.data = {
    ...node.data,
    label: nodeLabel(node, type), // Set the label based on the type
  };
  if (type === 'state') {
    node.type = 'output';
    node.style = {
          backgroundColor: 'lightblue',
          padding: '10px',
          whiteSpace: 'pre-wrap',
          minWidth: '150px',
          textAlign: 'center',
        };
    node.sourcePosition = 'right';
    node.targetPosition = 'left';
  }
  else if (type === 'trigger') {
    node.type = 'input';
    node.style = {
          whiteSpace: 'pre-wrap',
          padding: '10px',
        };
    node.sourcePosition = 'right';
    node.targetPosition = 'left';

  }
  else if (type === 'operator') {
    node.type = 'process';
    node.sourcePosition = 'right';
    node.targetPosition = 'left';
    node.style = {
          backgroundColor: 'lightyellow',
          padding: '10px',
          width: getNodeWidth(node.data.label), // Dynamically set width based on label
        };
  }
  return node;
};

export const nodeLabel = (node, type) => {
  console.log("Generating label for node:", node, "Type:", type);
  if (type === 'state') {
    return `${node.stateData.id} \n(Priority: ${node.stateData.priority})`;
  } else if  (type === 'trigger') {
    // Handling different trigger types
    let triggerLabel = node.trigger.id;
    if (node.trigger.time_trigger) {
      triggerLabel += ` \n(Time: ${node.trigger.time_trigger.last_change})`;
    } else if (node.trigger.action_trigger) {
      triggerLabel += ` \n(Action: ${node.trigger.action_trigger})`;
    } else if (node.trigger.topic_trigger) {
      triggerLabel += ` \n(Topic: ${node.trigger.topic_trigger})`;
    } else if (node.trigger.location_trigger) {
      triggerLabel += ` \n(Location: ${node.trigger.location_trigger})`;
    }
    return triggerLabel;
  } else if (type === 'operator') {
    return `${node.operatorType.toUpperCase()}`;
  }
};

/**
 * Converts a state machine description object to a YAML string.
 * @param {Object} description - The state machine description.
 * @returns {string} YAML string.
 */
export function createYAMLFromElements(description) {
  try {

    const yaml_description = {
      state_machines: description.state_machines.map(sm => ({
        id: sm.id,
        states: sm.states,
      })),
      states: description.states.map(state => ({
        id: state.id,
        priority: state.priority,
        trigger: state.trigger, // Assuming trigger is already in the correct format
        blocked_topics: state.blocked_topics || [], // Ensure blocked_topics is an array
      })),
    };

    // Convert to YAML
    console.log("Converting to YAML:", yaml_description);
    return yaml_description ? yaml.dump(yaml_description) : '';
  } catch (e) {
    console.error('Error converting to YAML:', e);
    return '';
  }
}

/**
 * Inverse of createElementsFromYAML: reconstructs the YAML description from nodes and edges.
 */
export function updateDescriptionFromElements(nodes, edges) {
  // 1. Find all state nodes
  const stateNodes = nodes.filter(n => n.nodeType === 'state');
  // 2. Find all trigger nodes
  const triggerNodes = nodes.filter(n => n.nodeType === 'trigger');
  // 3. Find all operator nodes
  const operatorNodes = nodes.filter(n => n.nodeType === 'operator');

  // 4. Get all unique state machine names from state node parents
  const stateMachineNames = [
    ...new Set(stateNodes.map(state => state.parent).filter(Boolean))
  ];

  // Helper: recursively reconstruct triggers/operators for a given state node
  function reconstructTrigger(targetNodeId) {
    // Find all incoming edges to this node
    const incomingEdges = edges.filter(e => e.target === targetNodeId);

    // For each incoming edge, get the source node
    const triggers = incomingEdges.map(edge => {
      const sourceNode = nodes.find(n => n.id === edge.source);
      if (!sourceNode) return null;

      if (sourceNode.nodeType === 'trigger') {
        return sourceNode.trigger;
      } else if (sourceNode.nodeType === 'operator') {
        // Recursively reconstruct nested triggers for this operator
        const nestedTriggers = reconstructTrigger(sourceNode.id);
        return {
          [sourceNode.operatorType]: Array.isArray(nestedTriggers) ? nestedTriggers : [nestedTriggers]
        };
      }
      return null;
    }).filter(Boolean);

    if (triggers.length === 0) return null;
    if (triggers.length === 1) return triggers[0];
    return triggers;
  }

  // Build state_machines array from parent names
  const state_machines = stateMachineNames.map(smName => ({
    id: smName,
    states: stateNodes
      .filter(state => state.parent === smName)
      .map(state => state.stateData.id),
  }));

  // Build states array
  const states = stateNodes.map(state => {
    const trigger = reconstructTrigger(state.id);
    return {
      id: state.stateData.id,
      priority: state.stateData.priority,
      trigger,
      blocked_topics: state.stateData.blocked_topics || [],
    };
  });

  return {
    state_machines,
    states,
  };
}