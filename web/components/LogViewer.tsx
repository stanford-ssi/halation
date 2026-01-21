"use client";

import { useEffect, useState, useCallback, useRef } from "react";
import ROSLIB from "roslib";

interface RosLog {
  stamp: { sec: number; nanosec: number };
  level: number;
  name: string;
  msg: string;
  file: string;
  function: string;
  line: number;
}

interface LogEntry {
  timestamp: string;
  level: string;
  node: string;
  message: string;
}

const LOG_LEVELS: Record<number, { label: string; color: string }> = {
  10: { label: "DEBUG", color: "text-gray-500" },
  20: { label: "INFO", color: "text-blue-600" },
  30: { label: "WARN", color: "text-yellow-600" },
  40: { label: "ERROR", color: "text-red-600" },
  50: { label: "FATAL", color: "text-red-800 font-bold" },
};

export function LogViewer({
  ros,
  isConnected,
}: {
  ros: ROSLIB.Ros | null;
  isConnected: boolean;
}) {
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [filter, setFilter] = useState<string>("");
  const [minLevel, setMinLevel] = useState<number>(20);
  const containerRef = useRef<HTMLDivElement>(null);
  const isAtBottomRef = useRef(true);

  const handleScroll = () => {
    if (containerRef.current) {
      const { scrollTop, scrollHeight, clientHeight } = containerRef.current;
      isAtBottomRef.current = scrollHeight - scrollTop - clientHeight < 30;
    }
  };

  const addLog = useCallback((rosLog: RosLog) => {
    const levelInfo = LOG_LEVELS[rosLog.level] || { label: "UNKNOWN", color: "text-gray-400" };
    const timestamp = new Date(
      rosLog.stamp.sec * 1000 + rosLog.stamp.nanosec / 1000000
    ).toLocaleTimeString();

    const entry: LogEntry = {
      timestamp,
      level: levelInfo.label,
      node: rosLog.name,
      message: rosLog.msg,
    };

    setLogs((prev) => [...prev.slice(-199), entry]);
  }, []);

  useEffect(() => {
    if (!ros || !isConnected) return;

    const rosoutTopic = new ROSLIB.Topic({
      ros,
      name: "/rosout",
      messageType: "rcl_interfaces/msg/Log",
    });

    rosoutTopic.subscribe((message) => {
      addLog(message as unknown as RosLog);
    });

    return () => {
      rosoutTopic.unsubscribe();
    };
  }, [ros, isConnected, addLog]);

  useEffect(() => {
    if (containerRef.current && isAtBottomRef.current) {
      containerRef.current.scrollTop = containerRef.current.scrollHeight;
    }
  }, [logs]);

  const filteredLogs = logs.filter((log) => {
    const levelNum = Object.entries(LOG_LEVELS).find(
      ([, v]) => v.label === log.level
    )?.[0];
    if (levelNum && parseInt(levelNum) < minLevel) return false;
    if (filter && !log.message.toLowerCase().includes(filter.toLowerCase()) &&
        !log.node.toLowerCase().includes(filter.toLowerCase())) return false;
    return true;
  });

  const getLevelColor = (level: string) => {
    const entry = Object.values(LOG_LEVELS).find((l) => l.label === level);
    return entry?.color || "text-gray-400";
  };

  return (
    <div className="mb-6">
      <h2 className="text-lg font-semibold mb-2">Node Logs</h2>

      <div className="flex gap-2 mb-2">
        <input
          type="text"
          placeholder="Filter logs..."
          value={filter}
          onChange={(e) => setFilter(e.target.value)}
          className="px-3 py-1 border rounded text-sm flex-1"
        />
        <select
          value={minLevel}
          onChange={(e) => setMinLevel(parseInt(e.target.value))}
          className="px-3 py-1 border rounded text-sm"
        >
          <option value={10}>DEBUG+</option>
          <option value={20}>INFO+</option>
          <option value={30}>WARN+</option>
          <option value={40}>ERROR+</option>
        </select>
        <button
          onClick={() => setLogs([])}
          className="px-3 py-1 rounded text-sm bg-red-100 text-red-800 hover:bg-red-200"
        >
          Clear
        </button>
      </div>

      <div ref={containerRef} onScroll={handleScroll} className="bg-gray-900 text-gray-100 rounded p-3 max-h-64 overflow-y-auto font-mono text-xs">
        {filteredLogs.length === 0 ? (
          <div className="text-gray-500">
            {isConnected
              ? "Waiting for logs from /rosout..."
              : "Not connected to ROS"}
          </div>
        ) : (
          filteredLogs.map((log, i) => (
            <div key={i} className="py-0.5 border-b border-gray-800">
              <span className="text-gray-500">{log.timestamp}</span>
              <span className={`ml-2 ${getLevelColor(log.level)}`}>[{log.level}]</span>
              <span className="ml-2 text-purple-400">[{log.node}]</span>
              <span className="ml-2 text-gray-100">{log.message}</span>
            </div>
          ))
        )}
      </div>
    </div>
  );
}
