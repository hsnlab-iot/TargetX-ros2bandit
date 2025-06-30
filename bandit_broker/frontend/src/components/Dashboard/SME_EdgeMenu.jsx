import React from "react";

const SME_EdgeMenu = ({ edgeMenu, onDelete, onCancel }) => {
  if (!edgeMenu.edgeId) return null;

  return (
    <div
      style={{
        position: 'fixed',
        top: edgeMenu.position.y,
        left: edgeMenu.position.x,
        background: 'white',
        border: '1px solid #ccc',
        padding: '8px',
        zIndex: 1000
      }}
    >
      <div>Edge: {edgeMenu.edgeId}</div>
      <button
        onClick={onDelete}
        style={{
          background: '#e57373',
          color: 'white',
          border: 'none',
          padding: '6px 12px',
          borderRadius: '4px',
          marginRight: 8
        }}
      >
        Delete
      </button>
      <button onClick={onCancel}>Cancel</button>
    </div>
  );
};

export default SME_EdgeMenu;