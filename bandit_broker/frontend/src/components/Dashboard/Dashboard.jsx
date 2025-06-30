import {
  Chart as ChartJS,
  ArcElement,
  LineElement,
  CategoryScale,
  LinearScale,
  PointElement,
  Tooltip,
  Legend,
} from 'chart.js';

ChartJS.register(
  LineElement,
  ArcElement,
  CategoryScale,
  LinearScale,
  PointElement,
  Tooltip,
  Legend
);

import React, { useState, useEffect, useRef } from 'react';
import Split from 'react-split';
import { Line } from 'react-chartjs-2';
import TopicsView from './TopicsView';
import ActionsTable from './ActionsTable';
import StateMachines from './StateMachines';
import StateMachinesEditor from './StateMachinesEditor';
import { formatBandwidth, updateShapeClass } from './utils';

const MENU_ITEMS = ['Topics Chart', 'Topics Table', 'Actions Table', 'State Machines', 'State Machines Editor'];


export default function Dashboard() {
  useEffect(() => {
    document.title = "ROS2Bandit Dashboard";
  }, []);

  const [selectedMenu, setSelectedMenu] = useState('Topics Chart');
  const [topicTraffic, setTopicTraffic] = useState({});
  const [blockedTopicTraffic, setBlockedTopicTraffic] = useState({});
  const [selectedTopics, setSelectedTopics] = useState({});
  const [actions, setActions] = useState([]);
  const [actionsLoaded, setActionsLoaded] = useState(false);
  const [svgContent, setSvgContent] = useState('');
  const [svgMap, setSvgMap] = useState({});
  const [totalBw, setTotalBw] = useState(0);  // State for total bandwidth
  const [totalBlockedBw, setTotalBlockedBw] = useState(0);  // State for total blocked bandwidth
  const [totalBwHistory, setTotalBwHistory] = useState([]); // History for total bandwidth
  const [totalBlockedBwHistory, setTotalBlockedBwHistory] = useState([]); // History for total blocked bandwidth

  const eventSourceRef = useRef(null);

  const topicsTableData = React.useMemo(() => 
    Object.entries(topicTraffic).map(([topic, info]) => ({
      topic,
      bw: info.bw,
      blockedBw: blockedTopicTraffic[topic]?.bw || 0,
      hz: info.hz,
    })), 
    [topicTraffic, blockedTopicTraffic]
  );

  useEffect(() => {
    if (selectedMenu === 'Actions Table' && !actionsLoaded) {
      fetch('/api/get-actions')
        .then(res => res.json())
        .then(data => {
          const formatted = Object.entries(data).map(([name, node]) => ({ name, node }));
          setActions(formatted);
          setActionsLoaded(true);
        })
        .catch(err => console.error('Failed to fetch actions', err));
    }
  }, [selectedMenu, actionsLoaded]);

  useEffect(() => {
    if (selectedMenu === 'State Machines') {
      fetch('/api/get-statemachines-diagram')
        .then(res => res.json())
        .then(data => setSvgMap(data))
        .catch(err => console.error('Failed to fetch SVGs', err));
    }
  }, [selectedMenu]);

  useEffect(() => {
    if (selectedMenu !== 'State Machines') return;
  
    const es = new EventSource('/api/stream-states');
  
    es.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        const updates = [];
  
        // Collect states
        for (const [id, active] of Object.entries(data["data-states"] || {})) {
          updates.push({ id, type: 'state', active });
        }
  
        // Collect triggers
        for (const [id, active] of Object.entries(data["data-triggers"] || {})) {
          updates.push({ id, type: 'trigger', active });
        }
  
        requestAnimationFrame(() => {
          updates.forEach(({ id, type, active }) => {
            updateShapeClass(id, type, active);
          });
        });
      } catch (err) {
        console.error('Error parsing state machine event:', err);
      }
    };
  
    return () => es.close();
  }, [svgContent, selectedMenu]);

  useEffect(() => {
    const trafficStream = new EventSource('/api/stream-topic-traffic');
    trafficStream.onmessage = (event) => {
      const parsed = JSON.parse(event.data);
      const allowed = { ...topicTraffic };   // Copy existing state to preserve unmodified topics
      const blocked = { ...blockedTopicTraffic };
      let currentMaxUpdateTime = 0
  
      // Iterate over the incoming parsed data and update traffic information
      for (const [topic, info] of Object.entries(parsed)) {
        // Update the maximum update time to the most recent one
        if (info.update > currentMaxUpdateTime) {
          currentMaxUpdateTime = info.update;
        }
       }
  
       for (const [topic, info] of Object.entries(parsed)) {
        const updateTime = info.update;
        // If topic is blocked, reset its bandwidth
        if (info.blocked) {
          blocked[topic] = info;
          allowed[topic] = { ...info, bw: 0 };  // Reset bw for unblocked topics
        } else {
          allowed[topic] = info;
          blocked[topic] = { ...info, bw: 0 };  // Reset bw for blocked topics
        }

        // Check if the topic exists in topicTraffic (new topic) and select it
        if (!(topic in topicTraffic)) {
          setSelectedTopics((prev) => {
            return { ...prev, [topic]: true };  // Mark new topic as selected
          });
        }
  
        // Check if the topic's update time exceeds 1.5 seconds since the most recent update
        const timeDiff = currentMaxUpdateTime - updateTime;
  
        //if (timeDiff > 1.5) {  // If update is older than 1.5 seconds
        //  allowed[topic] = { ...info, bw: 0 };  // Set bw to 0
        //  blocked[topic] = { ...info, bw: 0 };  // Set blocked bw to 0
        //}
      }
    
      // Update the state with the updated topic and blocked data
      setTopicTraffic(allowed);
      setBlockedTopicTraffic(blocked);
    };
    return () => trafficStream.close();
  }, [topicTraffic, blockedTopicTraffic, selectedTopics]);
  
  // Calculate the total bandwidth and blocked bandwidth for selected topics
  useEffect(() => {
    let totalBwValue = 0;
    let totalBlockedBwValue = 0;

    // Sum up the bandwidth and blocked bandwidth for selected topics
    Object.keys(selectedTopics).forEach((topic) => {
      if (selectedTopics[topic]) {
        totalBwValue += (topicTraffic[topic]?.bw || 0) * 8;  // Multiply by 8 to convert bytes to bits
        totalBlockedBwValue += (blockedTopicTraffic[topic]?.bw || 0) * 8;  // Multiply by 8 to convert bytes to bits
      }
    });

    setTotalBw(totalBwValue);
    setTotalBlockedBw(totalBlockedBwValue);

    // Add the current sums to the history
    setTotalBwHistory((prev) => [...prev, totalBwValue].slice(-50));  // Keep the last 50 values
    setTotalBlockedBwHistory((prev) => [...prev, totalBlockedBwValue].slice(-50));  // Keep the last 50 values
  }, [selectedTopics, topicTraffic, blockedTopicTraffic]);  // Recalculate when selectedTopics or topicTraffic updates

  // Graph data for total bandwidth and blocked bandwidth (stacked)
  const graphChartData = {
    labels: Array.from({ length: totalBwHistory.length }, (_, i) => i + 1),
    datasets: [
      {
        label: 'Actual Bandwidth (Bps)',
        data: totalBwHistory,
        fill: true,
        backgroundColor: 'rgba(0, 0, 255, 0.2)',
        borderColor: 'blue',
        tension: 0.2,
        stack: 'Stack 0',
      },
      {
        label: 'Unlocked Bandwidth (Bps)',
        data: totalBlockedBwHistory,
        fill: true,
        backgroundColor: 'rgba(255, 0, 0, 0.2)',
        borderColor: 'red',
        tension: 0.2,
        stack: 'Stack 0',
      },
    ],
  };

  return (
    <div className="flex flex-col h-screen overflow-hidden">
      <div className="flex-none bg-gray-100 p-2 shadow-md">
        {MENU_ITEMS.map(item => (
          <button
            key={item}
            onClick={() => setSelectedMenu(item)}
            className={`mr-2 px-4 py-2 rounded ${selectedMenu === item ? 'bg-blue-600 text-white' : 'bg-gray-200 text-black'}`}
          >
            {item}
          </button>
        ))}
      </div>

      <Split className="split flex flex-col flex-grow h-full" direction="vertical" sizes={[66, 34]} minSize={100} gutterSize={8}>
        <div className="flex flex-col h-full overflow-auto p-4 bg-white shadow-md rounded-lg">
          {selectedMenu === 'Topics Chart' || selectedMenu === 'Topics Table' ? (
            <TopicsView
              selectedMenu={selectedMenu}
              topicTraffic={topicTraffic}
              blockedTopicTraffic={blockedTopicTraffic}
              topicsTableData={Object.entries(topicTraffic).map(([topic, info]) => ({
                topic,
                bw: info.bw,
                blockedBw: blockedTopicTraffic[topic]?.bw || 0,
                hz: info.hz,
              }))}
              selectedTopics={selectedTopics}
              setSelectedTopics={setSelectedTopics}
            />
          ) : selectedMenu === 'State Machines' ? (
            <div style={{ display: 'flex', gap: '1rem', flexWrap: 'wrap' }}>
              {Object.entries(svgMap).map(([smName, svg]) => (
                <div key={smName} dangerouslySetInnerHTML={{ __html: svg }} />
              ))}
            </div>
          ) : selectedMenu === 'State Machines Editor' ? (
            <StateMachinesEditor 
              allTopics={Object.keys(topicTraffic)}
              allActions={actions.map(a => a.name)}
            />
          ) : selectedMenu === 'Actions Table' ? (
            <ActionsTable data={actions} />
          ) : (
            <div>Unknown view selected</div>
          )}
        </div>

        <div className="overflow-hidden bg-gray-200 border-t">
          <div className="p-4 h-full">
            <div className="bg-white border rounded-lg p-4 shadow-md h-full">
              <Line 
                data={graphChartData} 
                options={{
                  maintainAspectRatio: false,
                  animation: {
                    duration: 0, // Disable animation by setting duration to 0
                  },
                  scales: {
                    y: { 
                      stacked: true,
                      beginAtZero: true // Always start y-axis from zero
                    },
                    x: { stacked: true },
                  },
                }} 
              />
            </div>
          </div>
        </div>
      </Split>
    </div>
  );
}
