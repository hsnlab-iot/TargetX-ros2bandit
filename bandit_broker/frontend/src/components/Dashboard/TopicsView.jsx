import React, { useMemo, useCallback, useState, useEffect } from 'react';
import { Pie } from 'react-chartjs-2';
import { useReactTable, getCoreRowModel, getSortedRowModel, getFilteredRowModel, flexRender } from '@tanstack/react-table';
import { formatBandwidth } from './utils';

export default function TopicsView({ selectedMenu, topicTraffic, blockedTopicTraffic, topicsTableData, selectedTopics, setSelectedTopics }) {
  const [isSorting, setIsSorting] = useState(false);
  const [stableTableData, setStableTableData] = useState(topicsTableData);

  // Filter topicTraffic and blockedTopicTraffic based on selectedTopics
  const filteredTopicTraffic = Object.keys(topicTraffic)
    .filter((topic) => selectedTopics[topic])  // Only keep selected topics
    .reduce((obj, key) => {
      obj[key] = topicTraffic[key];
      return obj;
    }, {});

  const filteredBlockedTopicTraffic = Object.keys(blockedTopicTraffic)
    .filter((topic) => selectedTopics[topic])  // Only keep selected topics
    .reduce((obj, key) => {
      obj[key] = blockedTopicTraffic[key];
      return obj;
    }, {});

  // Pie Chart Data for Bandwidth, Blocked Bandwidth, and Message Rate
  const bwPieChartData = {
    labels: Object.keys(filteredTopicTraffic),
    datasets: [{
      label: 'Bandwidth',
      data: Object.values(filteredTopicTraffic).map(t => t.bw),
      backgroundColor: ['#4dc9f6', '#f67019', '#f53794', '#537bc4', '#acc236', '#166a8f', '#00a950', '#58595b', '#8549ba'],
    }],
  };

  const bbwPieChartData = {
    labels: Object.keys(filteredBlockedTopicTraffic),
    datasets: [{
      label: 'Blocked Bandwidth',
      data: Object.values(filteredBlockedTopicTraffic).map(t => t.bw),
      backgroundColor: ['#4dc9f6', '#f67019', '#f53794', '#537bc4', '#acc236', '#166a8f', '#00a950', '#58595b', '#8549ba'],
    }],
  };

  const hzPieChartData = {
    labels: Object.keys(filteredTopicTraffic),
    datasets: [{
      label: 'Message Rate',
      data: Object.values(filteredTopicTraffic).map(t => t.hz),
      backgroundColor: ['#4dc9f6', '#f67019', '#f53794', '#537bc4', '#acc236', '#166a8f', '#00a950', '#58595b', '#8549ba'],
    }],
  };

  const pieOptions = {
    plugins: {
      tooltip: {
        callbacks: {
          label: function (context) {
            const value = context.raw;
            const label = context.label;
            const datasetLabel = context.dataset.label || '';
            if (datasetLabel.toLowerCase().includes('bandwidth')) {
              return `${label}: ${formatBandwidth(value)}`;
            }
            return `${label}: ${typeof value === 'number' ? value.toFixed(2) : value}`;
          }
        }
      }
    }
  };

  // Memoize toggleTopicSelection so it doesn't change on every render
  const toggleTopicSelection = useCallback((topic) => {
    console.log(`Toggling selection for topic: ${topic}`);
    setSelectedTopics((prev) => {
      const newSelections = { ...prev };
      newSelections[topic] = !newSelections[topic];
      return newSelections;
    });
  }, [setSelectedTopics]);

  // Memoize columns, and include selectedTopics in the dependency array
  const columns = useMemo(() => [
    {
      id: 'select',
      header: 'Select',
      cell: info => {
        const topic = info.row.original.topic;
        const isSelected = !!selectedTopics[topic];
        return (
          <div onMouseDown={(e) => {
            e.preventDefault();
            toggleTopicSelection(topic);
          }}>
            <input
              type="checkbox"
              id={`select-${topic}`}
              name={`select-${topic}`}
              checked={isSelected}
              readOnly
            />
          </div>
        );
      },
      enableSorting: false,
    },
    { accessorKey: 'topic', header: 'Topic', cell: info => info.getValue() },
    { accessorKey: 'bw', header: 'Bandwidth', cell: info => formatBandwidth(info.getValue()) },
    { accessorKey: 'blockedBw', header: 'Blocked Bandwidth', cell: info => formatBandwidth(info.getValue()) },
    { accessorKey: 'hz', header: 'Message Rate (Hz)', cell: info => info.getValue()?.toFixed(2) ?? '' },
  // eslint-disable-next-line react-hooks/exhaustive-deps
  ], [toggleTopicSelection, selectedTopics]); // <-- add selectedTopics here

  const table = useReactTable({
    data: stableTableData, // <-- use the stable, possibly paused data
    columns,
    getCoreRowModel: getCoreRowModel(),
    getSortedRowModel: getSortedRowModel(),
    getFilteredRowModel: getFilteredRowModel(),
    initialState: { sorting: [{ id: 'bw', desc: true }] },
  });

  useEffect(() => {
    if (!isSorting) {
      console.log('Updating stableTableData with topicsTableData');
      setStableTableData(topicsTableData);
    }
  }, [topicsTableData, isSorting]);

  return (
    <div>
      {/* Charts */}
      {selectedMenu === 'Topics Chart' && (
        <div className="flex gap-6 h-full max-h-[80vh]">
          <div className="flex-1 overflow-hidden">
            <h3 className="text-md font-semibold mb-2">Bandwidth [Bps]</h3>
            <div style={{ height: '400px' }}>  {/* Set a fixed height for the pie chart container */}
              <Pie data={bwPieChartData} options={{ ...pieOptions, maintainAspectRatio: false }} />
            </div>
          </div>

          <div className="flex-1 overflow-hidden">
            <h3 className="text-md font-semibold mb-2">Blocked Bandwidth [Bps]</h3>
            <div style={{ height: '400px' }}>  {/* Set a fixed height for the pie chart container */}
              <Pie data={bbwPieChartData} options={{ ...pieOptions, maintainAspectRatio: false }} />
            </div>
          </div>

          <div className="flex-1 overflow-hidden">
            <h3 className="text-md font-semibold mb-2">Message Rate (Hz)</h3>
            <div style={{ height: '400px' }}>  {/* Set a fixed height for the pie chart container */}
              <Pie data={hzPieChartData} options={{ ...pieOptions, maintainAspectRatio: false }} />
            </div>
          </div>
        </div>
      )}

      {/* Topics Table */}
      {selectedMenu === 'Topics Table' && (
        <div>
          <div className="overflow-x-auto">
            <table className="min-w-full table-auto border-collapse border">
              <thead className="bg-gray-100">
                {table.getHeaderGroups().map(headerGroup => (
                  <tr key={headerGroup.id}>
                    {headerGroup.headers.map(header => (
                      <th
                        key={header.id}
                        onClick={(e) => {
                          console.log(`Sorting triggered for column: ${header.column.id}`);
                          setIsSorting(true);
                          setTimeout(() => {
                            setIsSorting(false);
                            console.log('isSorting set back to false');
                          }, 100);                          
                          header.column.getToggleSortingHandler()(e); // Trigger sorting
                        }}
                        className="p-2 border cursor-pointer"
                      >
                        {flexRender(header.column.columnDef.header, header.getContext())}
                        {header.column.getIsSorted() === 'asc' ? ' 🔼' : header.column.getIsSorted() === 'desc' ? ' 🔽' : ''}
                      </th>
                    ))}
                  </tr>
                ))}
              </thead>
              <tbody>
                {table.getRowModel().rows.map(row => (
                  <tr className="hover:bg-gray-100">
                    {row.getVisibleCells().map(cell => (
                      <td className="p-2 border">
                        {flexRender(cell.column.columnDef.cell, cell.getContext())}
                      </td>
                    ))}
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </div>
      )}
    </div>
  );
}
