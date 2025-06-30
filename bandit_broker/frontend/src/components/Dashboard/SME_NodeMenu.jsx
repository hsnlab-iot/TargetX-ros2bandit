import React, { useEffect, useState } from "react";

const SME_NodeMenu = ({
  menuInfo,
  menuDraft,
  setMenuDraft,
  node,
  onOk,
  onCancel,
  onDelete,
  allTopics,
  allActions,
}) => {
  // State to control modal and selection
  const [showBlockedTopicsSelectorLocal, setShowBlockedTopicsSelector] = useState(false);
  const [blockedTopicsSelection, setBlockedTopicsSelection] = React.useState([]);

  if (!menuInfo.nodeId) return null;

  // Render fields based on node type
  const renderFields = () => {
    if (!node) return null;
    if (node.nodeType === "operator") {
      return (
        <div>
          <div>Operation Type:</div>
          <select
            value={menuDraft?.operatorType || "OR"}
            onChange={e =>
              setMenuDraft(draft => ({ ...draft, operatorType: e.target.value }))
            }
          >
            <option value="or">OR</option>
            <option value="and">AND</option>
            <option value="not">NOT</option>
          </select>
        </div>
      );
    }
    if (node.nodeType === "trigger") {
      if (
        menuDraft &&
        !menuDraft.triggerType
      ) {
        let triggerType = "";
        if (menuDraft.trigger?.location_trigger) triggerType = "location";
        else if (menuDraft.trigger?.topic_trigger) triggerType = "topic";
        else if (menuDraft.trigger?.action_trigger) triggerType = "action";
        else if (menuDraft.trigger?.time_trigger) triggerType = "time";
        if (triggerType) {
          // This will cause a re-render, but only once per menu open
          setMenuDraft(draft => ({
            ...draft,
            triggerType,
          }));
          // Return null to prevent rendering until triggerType is set
          return null;
        }
      }
      console.log("Rendering trigger node fields", menuDraft);
      return (
        <div>
          <div>Name:</div>
          <input
            type="text"
            value={menuDraft?.trigger.id || ""}
            onChange={e =>
              setMenuDraft(draft => ({ ...draft, trigger: { ...draft.trigger, id: e.target.value } }))
            }
          />
          <div>Type:</div>
          <select
            value={menuDraft?.triggerType || ""}
            onChange={e => {
              const newType = e.target.value;
              setMenuDraft(draft => {
                let trigger = { ...draft.trigger };
                if (newType === "time" && !trigger.time_trigger) {
                  trigger.time_trigger = { last_change: "", in_state: "" };
                }
                // Optionally, clear other trigger types here if needed
                return { ...draft, triggerType: newType, trigger };
              });
            }}
          >
            <option value="">Select</option>
            <option value="location">location</option>
            <option value="topic">topic</option>
            <option value="action">action</option>
            <option value="time">time</option>
          </select>

          {/* Dynamic fields based on triggerType */}
          {menuDraft?.triggerType === "location" && (
            <div>
              <div>Location:</div>
              <input
                type="text"
                value={menuDraft?.trigger.location_trigger || ""}
                onChange={e =>
                  setMenuDraft(draft => ({ ...draft, trigger: { ...draft.trigger, location_trigger: e.target.value } }))
                }
              />
            </div>
          )}

          {menuDraft?.triggerType === "topic" && (
            <div>
              <div>Topic:</div>
              <select
                value={menuDraft?.trigger.topic_trigger || ""}
                onChange={e =>
                  setMenuDraft(draft => ({ ...draft, trigger: { ...draft.trigger, topic_trigger: e.target.value } }))
                }
              >
                <option value="">Select topic</option>
                {(allTopics || []).map(topic => (
                  <option key={topic} value={topic}>{topic}</option>
                ))}
              </select>
            </div>
          )}

          {menuDraft?.triggerType === "action" && (
            <div>
              <div>Action:</div>
              <select
                value={menuDraft?.trigger?.action_trigger || ""}
                onChange={e =>
                  setMenuDraft(draft => ({
                    ...draft,
                    trigger: { ...draft.trigger, action_trigger: e.target.value }
                  }))
                }
              >
                <option value="">Select action</option>
                {(allActions || []).map(action => (
                  <option key={action} value={action}>{action}</option>
                ))}
              </select>
            </div>
          )}

          {menuDraft?.triggerType === "time" && (
            <div>
              <div>
                <label>
                  <input
                    type="checkbox"
                    checked={!!menuDraft?.trigger.time_trigger.last_change}
                    onChange={e =>
                      setMenuDraft(draft => ({ ...draft, time1Checked: e.target.checked }))
                    }
                  />
                  last_change
                </label>
                <input
                  type="number"
                  value={menuDraft?.trigger.time_trigger.last_change || ""}
                  onChange={e =>
                    setMenuDraft(draft => ({ ...draft, trigger: { ...draft.trigger, time_trigger: { ...draft.trigger.time_trigger, last_change: e.target.value } } }))
                  }
                  style={{ marginLeft: 8 }}
                />
              </div>
              <div>
                <label>
                  <input
                    type="checkbox"
                    checked={!!menuDraft?.trigger.time_trigger.in_state}
                    onChange={e =>
                      setMenuDraft(draft => ({ ...draft, time2Checked: e.target.checked }))
                    }
                  />
                  in_state
                </label>
                <input
                  type="number"
                  value={menuDraft?.trigger.time_trigger.in_state  || ""}
                  onChange={e =>
                    setMenuDraft(draft => ({ ...draft, trigger: { ...draft.trigger, time_trigger: { ...draft.trigger.time_trigger, in_state: e.target.value } } }))
                  }
                  style={{ marginLeft: 8 }}
                />
              </div>
            </div>
          )}
        </div>
      );
    }
    if (node.nodeType === "state") {
      console.log("Rendering state node fields", menuDraft);
      return (
        <div>
          <div>Name:</div>
          <input
            type="text"
            value={menuDraft?.stateData?.id || ""}
            onChange={e =>
              setMenuDraft(draft => ({
                ...draft,
                stateData: { ...draft.stateData, id: e.target.value }
              }))
            }
          />
          <div>Priority:</div>
          <input
            type="number"
            value={menuDraft?.stateData?.priority || ""}
            onChange={e =>
              setMenuDraft(draft => ({
                ...draft,
                stateData: { ...draft.stateData, priority: e.target.value }
              }))
            }
          />
        </div>
      );
    }
    return <div>Unknown node type</div>;
  };

  // When opening the selector, initialize selection with topics already in blocked_topics
  const handleBlockedTopicsOpen = () => {
    setBlockedTopicsSelection(menuDraft?.stateData?.blocked_topics || []);
    setShowBlockedTopicsSelector(true);
  };

  // When OK is pressed, update menuDraft.stateData.blocked_topics
  const handleBlockedTopicsOkLocal = () => {
    setMenuDraft(draft => ({
      ...draft,
      stateData: {
        ...draft.stateData,
        blocked_topics: blockedTopicsSelection
      }
    }));
    setShowBlockedTopicsSelector(false);
  };

  // When Cancel is pressed, just close the modal
  const handleBlockedTopicsCancelLocal = () => {
    setShowBlockedTopicsSelector(false);
  };

  // Combine allTopics and blocked_topics, removing duplicates
  const blockedTopics = menuDraft?.stateData?.blocked_topics || [];
  const allTopicsSet = new Set([...(allTopics || []), ...blockedTopics]);
  const topicsForSelection = Array.from(allTopicsSet);

  // 1. Do your final things here (e.g., validation, local state updates, logging)
  const handleMenuOk = () => {
    let cleanedDraft = { ...menuDraft };
    console.log("Initial cleaned draft:", cleanedDraft);

    // Clean up time trigger fields
    if (node?.nodeType === "trigger" && cleanedDraft?.triggerType === "time") {
      const time_trigger = { ...cleanedDraft.trigger.time_trigger };
      if (!time_trigger.last_change) {
        delete cleanedDraft.trigger.time_trigger.last_change;
      }
      if (!time_trigger.in_state) {
        delete cleanedDraft.trigger.time_trigger.in_state;
      }
    }
    console.log("After cleaning time trigger fields:", cleanedDraft);

    // Clean up trigger fields
    if (node?.nodeType === "trigger") {
      const triggerType = cleanedDraft.triggerType;
      let cleanedTrigger = {};
      if (triggerType === "location") {
        cleanedTrigger = { location_trigger: cleanedDraft.trigger.location_trigger };
      } else if (triggerType === "topic") {
        cleanedTrigger = { topic_trigger: cleanedDraft.trigger.topic_trigger };
      } else if (triggerType === "action") {
        cleanedTrigger = { action_trigger: cleanedDraft.trigger.action_trigger };
      } else if (triggerType === "time") {
        cleanedTrigger = { time_trigger: cleanedDraft.trigger.time_trigger };
      }
      // Remove unwanted fields from the root
      const { trigger, selectedAction, ...rest } = cleanedDraft;
      cleanedDraft = {
        ...rest,
        trigger: {
          ...cleanedTrigger,
          id: cleanedDraft.trigger.id,
        },
      };
    }

    console.log("Final cleaned draft before onOk:", cleanedDraft);
    // Now call onOk with the cleaned draft
    if (onOk) onOk(cleanedDraft);
  };

  return (
    <div
      style={{
        position: "fixed",
        top: menuInfo.position.y,
        left: menuInfo.position.x,
        background: "white",
        border: "1px solid #ccc",
        padding: "8px",
        zIndex: 1000,
      }}
    >
      {renderFields()}
      {/* Blocked Topics button only for state nodes */}
      {node?.nodeType === "state" && (
        <div style={{ margin: "16px 0 8px 0" }}>
          <button onClick={handleBlockedTopicsOpen}>Blocked Topics</button>
        </div>
      )}
      <div style={{ display: "flex", gap: 8 }}>
        <button onClick={handleMenuOk}>OK</button>
        <button onClick={onCancel}>Cancel</button>
        <button
          onClick={onDelete}
          style={{
            background: "#e57373",
            color: "white",
            border: "none",
            padding: "6px 12px",
            borderRadius: "4px",
          }}
        >
          Delete
        </button>
      </div>

      {/* The modal */}
      {showBlockedTopicsSelectorLocal && (
        <div
          style={{
            position: 'fixed',
            top: '30%',
            left: '40%',
            background: 'white',
            border: '1px solid #888',
            padding: '16px',
            zIndex: 2000
          }}
        >
          <div style={{ marginBottom: 8, fontWeight: 'bold' }}>Select Blocked Topics</div>
          <div style={{ maxHeight: 200, overflowY: 'auto' }}>
            {(topicsForSelection || []).map(topic => (
              <label key={topic} style={{ display: 'block' }}>
                <input
                  type="checkbox"
                  checked={blockedTopicsSelection.includes(topic)}
                  onChange={e => {
                    setBlockedTopicsSelection(sel =>
                      e.target.checked
                        ? [...sel, topic]
                        : sel.filter(t => t !== topic)
                    );
                  }}
                />
                {topic}
              </label>
            ))}
          </div>
          <div style={{ marginTop: 12, display: 'flex', gap: 8 }}>
            <button onClick={handleBlockedTopicsOkLocal}>OK</button>
            <button onClick={handleBlockedTopicsCancelLocal}>Cancel</button>
          </div>
        </div>
      )}
    </div>
  );
};

export default SME_NodeMenu;