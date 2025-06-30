import React from 'react';

export default function StateMachines({ svgContent }) {
  return (
    <div className="overflow-auto max-h-[66vh]">
      <h3 className="text-md font-semibold mb-2">State Machine Diagram</h3>
      <div className="border rounded shadow p-2 bg-white" dangerouslySetInnerHTML={{ __html: svgContent }} />
    </div>
  );
}
