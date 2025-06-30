import React, { useMemo, useState, useEffect, useRef } from 'react';
import ReactFlow, { MiniMap, Controls } from 'react-flow-renderer';
import dagre from 'dagre';
import * as yaml from 'js-yaml';
import { saveAs } from 'file-saver';
import { createElementsFromYAML } from './utils';
import { createYAMLFromElements } from './utils';
import { updateDescriptionFromElements } from './utils';
import { addNodeStyle } from './utils';
import { nodeLabel } from './utils';
import SME_EdgeMenu from './SME_EdgeMenu';
import SME_NodeMenu from './SME_NodeMenu';

const StateMachinesEditor = React.memo(
  ({ allTopics, allActions }) => {
    const [description, setDescription] = useState(null);
    const [nodes, setNodes] = useState([]);
    const [edges, setEdges] = useState([]);
    const [edgeUpdates, setEdgeUpdates] = useState([]);
    const [stateMachineNames, setStateMachineNames] = useState([]);
    const [selectedStateMachine, setSelectedStateMachine] = useState('');
    const [visibleNodes, setVisibleNodes] = useState([]);
    const [visibleEdges, setVisibleEdges] = useState([]);
    const [refreshLayout, setRefreshLayout] = useState(0);
    const [menuInfo, setMenuInfo] = useState({ nodeId: null, position: { x: 0, y: 0 } });
    const [menuDraft, setMenuDraft] = useState(null);
    const [edgeMenu, setEdgeMenu] = useState({ edgeId: null, position: { x: 0, y: 0 } });

    const prevNodesRef = useRef();
    const prevEdgesRef = useRef();

    // Utility function to compare arrays of objects by id
    const logDifferences = (prev, next, label) => {
      if (!prev) {
        console.log(`[${label}] Initial value:`, next);
        return;
      }
      if (JSON.stringify(prev) === JSON.stringify(next)) {
        // No change
        return;
      }
      console.log(`[${label}] changed.`);
      console.log("Previous:", prev);
      console.log("Next:", next);

      const prevIds = prev.map(item => item.id);
      const nextIds = next.map(item => item.id);
      const added = nextIds.filter(id => !prevIds.includes(id));
      const removed = prevIds.filter(id => !nextIds.includes(id));
      console.log(`Added ${label}:`, added);
      console.log(`Removed ${label}:`, removed);
    };

    // Handle file upload
    const handleFileUpload = (event) => {
      const file = event.target.files[0];
      if (file) {
        const reader = new FileReader();
        reader.onload = () => {
          try {
            const parsedData = yaml.load(reader.result); // Parse YAML file
            console.log("Parsed YAML Data:", parsedData);

            // Extract state machine names
            if (parsedData && Array.isArray(parsedData.state_machines)) {
              const names = parsedData.state_machines.map(sm => sm.id);
              setStateMachineNames(names);
              setSelectedStateMachine(names[0] || '');
            } else {
              setStateMachineNames([]);
              setSelectedStateMachine('');
            }

            setDescription(parsedData); // Update state with parsed data
          } catch (error) {
            console.error("Error reading or parsing the YAML file:", error);
          }
        };
        reader.readAsText(file);
      }
    };

    // Process YAML description and generate nodes/edges
    useEffect(() => {
      if (description) {
        console.log("Processing YAML data...");
        const { nodes, edges } = createElementsFromYAML(description); // Process YAML
        setNodes(nodes); // Update nodes
        setEdges(edges); // Update edges
      }
    }, [description]);

    // Apply Dagre layout whenever the nodes or edges change
    useEffect(() => {
      console.log("visibleNodes:", visibleNodes);
      console.log("visibleEdges:", visibleEdges);
      if (!visibleNodes.length || !visibleEdges.length) {
        console.warn("No visible nodes or edges to layout");
        return; // Exit early if nothing to layout
      }

      const g = new dagre.graphlib.Graph();
      g.setGraph({
        rankdir: 'LR', // Left to right layout
        nodesep: 50,
        edgesep: 10,
        compound: true,
      });
      g.setDefaultEdgeLabel(() => ({}));

      // Add only visible nodes to the graph
      visibleNodes.forEach((node) => {
        g.setNode(node.id, { width: 180, height: 60 });
        if (node.parentId) {
          g.setParent(node.id, node.parentId);
        }
      });

      // Add only visible edges to the graph
      visibleEdges.forEach((edge) => {
        g.setEdge(edge.source, edge.target);
      });

      dagre.layout(g);

      // Update positions only for visible nodes
      const updatedNodes = nodes.map((node) => {
        if (!visibleNodes.find(n => n.id === node.id)) return node;
        const dagreNode = g.node(node.id);
        if (!dagreNode) return node;
        return {
          ...node,
          position: { x: dagreNode.x, y: dagreNode.y },
        };
      });

      const positionsChanged = visibleNodes.some((node) => {
        const dagreNode = g.node(node.id);
        return (
          !dagreNode ||
          node.position.x !== dagreNode.x ||
          node.position.y !== dagreNode.y
        );
      });

      if (positionsChanged) {
        setNodes(updatedNodes);
      }
    }, [visibleNodes, visibleEdges, nodes, refreshLayout]); // <-- add refreshLayout here

    // Log differences in nodes and edges
    useEffect(() => {
      logDifferences(prevNodesRef.current, nodes, "Nodes");
      logDifferences(prevEdgesRef.current, edges, "Edges");
      prevNodesRef.current = nodes;
      prevEdgesRef.current = edges;
    }, [nodes, edges]);

    // Update visibleNodes and visibleEdges whenever nodes, edges, or selectedStateMachine changes
    useEffect(() => {
      const filteredNodes = nodes.filter(node => node.parent === selectedStateMachine);
      setVisibleNodes(filteredNodes);

      const visibleNodeIds = new Set(filteredNodes.map(node => node.id));
      const filteredEdges = edges.filter(
        edge => visibleNodeIds.has(edge.source) && visibleNodeIds.has(edge.target)
      );
      setVisibleEdges(filteredEdges);
    }, [nodes, edges, selectedStateMachine]);

    // Handle new edge connection
    const onConnect = (params) => {
      console.log("Creating new connection:", params);
      if (!params.source || !params.target) {
        console.warn("Invalid connection parameters:", params);
        return; // Exit if source or target is not defined
      }
      if ((params.source === params.target) ||
      params.source.type === "output" ||
      params.target.type === "input") {
        console.warn("Invalid connection: source and target cannot be the same or of incompatible types.");
        return; // Exit if source and target are the same or incompatible
      }
      const sourceNode = nodes.find(n => n.id === params.source);
      const targetNode = nodes.find(n => n.id === params.target);

      // Check if source is a process and already has an outgoing edge
      if (sourceNode?.type === "process") {
        const outgoing = edges.some(e => e.source === params.source);
        if (outgoing) {
          console.warn("A process node can only have one outgoing edge.");
          return;
        }
      }

      // Check if target is an output and already has an incoming edge
      if (targetNode?.type === "output") {
        const incoming = edges.some(e => e.target === params.target);
        if (incoming) {
          console.warn("An output node can only have one incoming edge.");
          return;
        }
      }
      setEdges((prevEdges) => [
        ...prevEdges,
        {
          id: `${params.source}-${params.target}`,
          source: params.source,
          target: params.target,
          animated: true,
          style: { strokeWidth: 4 }, // <-- bolder edge
          interactive: true, // Allow interaction with edges
        },
      ]);
    };

    // Handle file save
    const handleFileSave = () => {
      const description = updateDescriptionFromElements(nodes, edges);
      const yamlContent = createYAMLFromElements(description);
      const blob = new Blob([yamlContent], { type: "application/x-yaml" });
      saveAs(blob, "state-machine.yaml"); // Save the file as 'state-machine.yaml'
    };

    // Add this function to handle drag start
    const handleDragStart = (event, nodeType) => {
      console.log(`Dragging node type: ${nodeType}`);
      event.dataTransfer.setData("application/reactflow", nodeType);
      event.dataTransfer.effectAllowed = "move";
    };

    // Add this function to handle drop
    const handleDrop = (event) => {
      event.preventDefault();

      const reactFlowBounds = event.target.getBoundingClientRect();
      const position = {
        x: event.clientX - reactFlowBounds.left,
        y: event.clientY - reactFlowBounds.top,
      };

      const nodeType = event.dataTransfer.getData("application/reactflow");
      console.log(`Dropped node type: ${nodeType} at position:`, position);

      let  newNode = {
        id: `${nodeType}-${nodes.length + 1}`, // Unique ID
        position,
        parent: selectedStateMachine, // <-- Assign to current state machine
        nodeType: nodeType, // 'operation', 'trigger', or 'state'
      };
      if (nodeType === "state") {
        newNode = {
          ...newNode,
          stateData: { id: `new-state-${nodes.length + 1}`, priority: 99 }
        };
      } else if (nodeType === "trigger") {
        newNode = {
          ...newNode,
          trigger: { id: `new-trigger-${nodes.length + 1}`, timeTriggger: { last_change: 0 } }
        };
      } else if (nodeType === "operator") {
        newNode = {
          ...newNode,
          operatorType: "or"
        };
      }
      const styledNode = addNodeStyle(newNode, nodeType);
      setNodes((prevNodes) => [...prevNodes, styledNode]);
    };

    // Add this function to handle drag over
    const handleDragOver = (event) => {
      console.log("Dragging over React Flow container");
      event.preventDefault();
      event.dataTransfer.dropEffect = "move";
    };

    // Add this function to handle node click
    const handleNodeClick = (event, node) => {
      setMenuInfo({
        nodeId: node.id,
        position: { x: event.clientX, y: event.clientY }
      });
      setMenuDraft({ ...node }); // Start with current node data
    };

    const handleMenuOk = (cleanedDraft) => {
      setNodes(nodes =>
        nodes.map(n =>
          n.id === menuInfo.nodeId
            ? {
                ...n,
                ...cleanedDraft,
                data: {
                  ...n.data,
                  label: nodeLabel(cleanedDraft, n.nodeType)
                }
              }
            : n
        )
      );
      setMenuInfo({ nodeId: null, position: { x: 0, y: 0 } });
      setMenuDraft(null);
    };

    const handleMenuCancel = () => {
      setMenuInfo({ nodeId: null, position: { x: 0, y: 0 } });
      setMenuDraft(null);
    };

    const handleMenuDelete = () => {
      // Delete all edges connected to this node
      setEdges(edges => edges.filter(e => e.source !== menuInfo.nodeId && e.target !== menuInfo.nodeId));
      // Delete the node itself
      setNodes(nodes => nodes.filter(n => n.id !== menuInfo.nodeId));

      setMenuInfo({ nodeId: null, position: { x: 0, y: 0 } });
      setMenuDraft(null);
    };

    // Add this function to handle edge click
    const handleEdgeClick = (event, edge) => {
      event.stopPropagation();
      setEdgeMenu({
        edgeId: edge.id,
        position: { x: event.clientX, y: event.clientY }
      });
    };

    const handleEdgeDelete = () => {
      setEdges(edges => edges.filter(e => e.id !== edgeMenu.edgeId));
      setEdgeMenu({ edgeId: null, position: { x: 0, y: 0 } });
    };

    const handleEdgeMenuCancel = () => {
      setEdgeMenu({ edgeId: null, position: { x: 0, y: 0 } });
    };

    const handleNewStateMachine = () => {
      // Find the largest sm<number>
      const smNumbers = stateMachineNames
        .map(name => {
          const match = name.match(/^sm(\d+)$/);
          return match ? parseInt(match[1], 10) : null;
        })
        .filter(num => num !== null);
      const maxNum = smNumbers.length > 0 ? Math.max(...smNumbers) : 0;
      const newNum = maxNum + 1;
      const newName = `sm${newNum}`;

      setStateMachineNames(prev => [...prev, newName]);
      setSelectedStateMachine(newName);
    };

    const handleDeleteStateMachine = () => {
      if (!selectedStateMachine) return;

      // Remove all nodes and edges related to the selected state machine
      setNodes(prevNodes => prevNodes.filter(node =>
        node.parent !== selectedStateMachine && node.id !== selectedStateMachine
      ));
      setEdges(prevEdges => prevEdges.filter(edge => {
        const sourceNode = nodes.find(n => n.id === edge.source);
        const targetNode = nodes.find(n => n.id === edge.target);
        return (
          (sourceNode?.parent !== selectedStateMachine && sourceNode?.id !== selectedStateMachine) &&
          (targetNode?.parent !== selectedStateMachine && targetNode?.id !== selectedStateMachine)
        );
      }));

      setStateMachineNames(prev => {
        const updated = prev.filter(name => name !== selectedStateMachine);
        setSelectedStateMachine(updated[0] || "");
        return updated;
      });
    };

    return (
      <div style={{ height: "100vh", display: "flex" }}>
        {/* Left pane for buttons */}
        <div style={{ width: "200px", padding: "10px", background: "#f5f5f5" }}>
          <button onClick={handleFileSave}>Save File</button>
          <input type="file" accept=".yaml" onChange={handleFileUpload} />
          <div style={{ display: "flex", flexDirection: "column", gap: "16px", marginTop: "24px" }}>
            {/* State Machine Selection */}
            <label htmlFor="state-machine-select" style={{ fontWeight: "bold" }}>
              State Machine:
            </label>
            <select
              id="state-machine-select"
              style={{ marginBottom: "8px" }}
              value={selectedStateMachine}
              onChange={e => setSelectedStateMachine(e.target.value)}
            >
              {stateMachineNames.map(name => (
                <option key={name} value={name}>{name}</option>
              ))}
            </select>

            {/* New State Machine */}
            <button style={{ marginBottom: "8px" }} onClick={handleNewStateMachine}>New State Machine</button>

            {/* Delete State Machine */}
            <button style={{ marginBottom: "8px", color: "red" }} onClick={handleDeleteStateMachine}>Delete State Machine</button>

            {/* Refresh Layout */}
            <button
              style={{ marginTop: "16px" }}
              onClick={() => setRefreshLayout(v => v + 1)}
            >
              Refresh Layout
            </button>
          </div>
          <div style={{ marginTop: "20px", display: "flex", flexDirection: "column", gap: "10px" }}></div>
        </div>

        {/* Central pane for the state machine editor */}
        <div style={{ flexGrow: 1, height: "100vh", padding: "10px" }}>
          <ReactFlow
            nodes={visibleNodes}
            edges={[...visibleEdges, ...edgeUpdates]}
            style={{ background: "lightgrey", height: "100%", width: "100%" }}
            nodeTypes={{}}
            connectionLineStyle={{ stroke: "black", strokeWidth: 3 }}
            snapToGrid={true}
            snapGrid={[15, 15]} // Grid size
            fitView={true} // Fit the view
            onConnect={onConnect}
            onDrop={handleDrop} // Handle drop event
            onDragOver={handleDragOver} // Handle drag over event
            onNodeClick={handleNodeClick} // Handle node click event
            onEdgeClick={handleEdgeClick} // Handle edge click event
          >
            <MiniMap />
            <Controls />
          </ReactFlow>
        </div>

        {/* Right pane for draggable nodes */}
        <div style={{ width: "200px", padding: "10px", background: "#f5f5f5", overflowY: "auto" }}>
          <div
            style={{ padding: "10px", background: "#e0e0e0", marginBottom: "10px", cursor: "pointer" }}
            draggable
            onDragStart={(event) => handleDragStart(event, "state")}
          >
            State Node
          </div>
          <div
            style={{ padding: "10px", background: "#e0e0e0", marginBottom: "10px", cursor: "pointer" }}
            draggable
            onDragStart={(event) => handleDragStart(event, "trigger")}
          >
            Trigger Node
          </div>
          <div
            style={{ padding: "10px", background: "#e0e0e0", marginBottom: "10px", cursor: "pointer" }}
            draggable
            onDragStart={(event) => handleDragStart(event, "operator")}
          >
            Operator Node
          </div>
        </div>

        {/* Context menu for nodes */}
        {menuInfo.nodeId && (
          <div
            style={{
              position: 'fixed',
              top: menuInfo.position.y,
              left: menuInfo.position.x,
              background: 'white',
              border: '1px solid #ccc',
              padding: '8px',
              zIndex: 1000
            }}
          >
            <SME_NodeMenu
              menuInfo={menuInfo}
              menuDraft={menuDraft}
              setMenuDraft={setMenuDraft}
              node={nodes.find(n => n.id === menuInfo.nodeId)}
              onOk={handleMenuOk}
              onCancel={handleMenuCancel}
              onDelete={handleMenuDelete}
              onBlockedTopics={() => setShowTopicsSelector(true)}
              allTopics={allTopics} // Pass allTopics to SME_NodeMenu
              allActions={allActions} // Pass allActions to SME_NodeMenu
            />
          </div>
        )}

        {/* Context menu for edges */}
        {edgeMenu.edgeId && (
          <SME_EdgeMenu
            edgeMenu={edgeMenu}
            onDelete={handleEdgeDelete}
            onCancel={handleEdgeMenuCancel}
          />
        )}
      </div>
    );
  },
  (prevProps, nextProps) => {
    // Prevent re-render if description, nodes, and edges haven't changed
    return (
      prevProps.description === nextProps.description &&
      prevProps.nodes === nextProps.nodes &&
      prevProps.edges === nextProps.edges
    );
  }
);

export default StateMachinesEditor;
