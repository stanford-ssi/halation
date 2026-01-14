"use client";

import { useEffect, useState } from "react";
import ROSLIB from "roslib";
import { UsageChart } from "./UsageChart";

interface NodeUsage {
  name: string;
  namespace: string;
  pid: number;
  memory_mb: number;
  cpu_percent: number;
  threads: number;
  cmd: string;
}

interface UnmatchedProcess {
  pid: number;
  name: string;
  memory_mb: number;
  cpu_percent: number;
  threads: number;
  cmd: string;
}

interface UsageData {
  nodes: NodeUsage[];
  unmatched: UnmatchedProcess[];
  total_cpu: number;
  total_memory_mb: number;
}

interface UsageMonitorProps {
  ros: ROSLIB.Ros | null;
  isConnected: boolean;
  topics: string[];
}

interface UsageHistoryPoint {
  timestamp: string;
  time: string;
  totalCpu: number;
  totalMemory: number;
  nodes: NodeUsage[];
  unmatched: UnmatchedProcess[];
}

const STORAGE_KEY = "usageHistory";
const TIME_FORMAT: Intl.DateTimeFormatOptions = {
  hour: "numeric",
  minute: "2-digit",
  second: "2-digit",
};

function loadSavedHistory(): UsageHistoryPoint[] {
  if (typeof window !== "undefined") {
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved) {
      try {
        return JSON.parse(saved) as UsageHistoryPoint[];
      } catch {
        return [];
      }
    }
  }
  return [];
}

function usageDataFromHistory(history: UsageHistoryPoint[]): UsageData | null {
  const last = history.at(-1);
  if (!last) return null;
  return {
    nodes: last.nodes,
    unmatched: last.unmatched,
    total_cpu: last.totalCpu,
    total_memory_mb: last.totalMemory,
  };
}

export function UsageMonitor({ ros, isConnected, topics }: UsageMonitorProps) {
  const [usageHistory, setUsageHistory] = useState(loadSavedHistory);
  const [usageData, setUsageData] = useState<UsageData | null>(() =>
    usageDataFromHistory(loadSavedHistory())
  );
  const [allNodes, setAllNodes] = useState<string[]>([]);

  useEffect(() => {
    if (!isConnected || !ros) return;

    const usageTopic = new ROSLIB.Topic({
      ros,
      name: "/usage",
      messageType: "std_msgs/String",
    });

    usageTopic.subscribe((msg: unknown) => {
      try {
        const parsed = JSON.parse((msg as { data: string }).data) as UsageData;
        setUsageData(parsed);

        // Add to history for charting
        const now = new Date();
        const timeStr = now.toLocaleTimeString("en-US", TIME_FORMAT);

        setUsageHistory((prev) => {
          const newPoint: UsageHistoryPoint = {
            timestamp: now.toISOString(),
            time: timeStr,
            totalCpu: parsed.total_cpu,
            totalMemory: parsed.total_memory_mb,
            nodes: parsed.nodes,
            unmatched: parsed.unmatched,
          };
          // Keep last 300 points (10 minutes of data at 2s intervals)
          if (prev.length >= 300) {
            return [...prev.slice(1), newPoint];
          }
          return [...prev, newPoint];
        });
      } catch (error) {
        console.error("Failed to parse usage data:", error, msg);
      }
    });

    return () => usageTopic.unsubscribe();
  }, [isConnected, ros]);

  // Save usage history to localStorage when it changes
  useEffect(() => {
    if (usageHistory.length > 0) {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(usageHistory));
    }
  }, [usageHistory]);

  // Discover nodes from available topics (pattern: /node_name/topic_name)
  useEffect(() => {
    if (topics.length === 0) return;
    const nodes = new Set(
      topics
        .map((t) => t.split("/").filter(Boolean))
        .filter((parts) => parts.length > 1)
        .map((parts) => parts[0])
    );
    setAllNodes([...nodes]);
  }, [topics]);

  if (!isConnected || !usageData) {
    return (
      <div className="mb-6 p-4 bg-gray-100 rounded">
        <h2 className="text-lg font-semibold mb-2">System Usage</h2>
        <div className="text-gray-500">
          {!isConnected ? "Not connected to ROS" : "Waiting for usage data..."}
        </div>
      </div>
    );
  }

  const trackedNodeNames = new Set(usageData.nodes.map((n) => n.name));
  const missingNodes = allNodes.filter((node) => !trackedNodeNames.has(node));

  const sortedByCpu = [...usageData.nodes, ...usageData.unmatched].sort(
    (a, b) => b.cpu_percent - a.cpu_percent,
  );
  const sortedByMemory = [...usageData.nodes, ...usageData.unmatched].sort(
    (a, b) => b.memory_mb - a.memory_mb,
  );

  return (
    <div className="mb-6 mt-8">
      <h2 className="text-lg font-semibold mb-3">System Usage</h2>

      {/* Charts with integrated totals and top processes */}
      {usageHistory.length > 0 && (
        <div className="mb-4">
          <UsageChart
            data={usageHistory}
            currentCpu={usageData.total_cpu}
            currentMemory={usageData.total_memory_mb}
            topCpuProcesses={sortedByCpu}
            topMemoryProcesses={sortedByMemory}
          />
        </div>
      )}

      {/* Process Count and Missing Nodes */}
      <div className="mt-4 space-y-2">
        <div className="text-sm text-gray-600">
          Tracking {usageData.nodes.length} ROS nodes and{" "}
          {usageData.unmatched.length} unmatched processes
        </div>

        {missingNodes.length > 0 && (
          <div className="bg-yellow-50 border border-yellow-200 rounded p-3">
            <div className="text-sm font-semibold text-yellow-800 mb-1">
              Nodes without usage data:
            </div>
            <div className="flex flex-wrap gap-2">
              {missingNodes.map((node) => (
                <span
                  key={node}
                  className="px-2 py-1 bg-yellow-100 text-yellow-800 rounded text-xs font-mono"
                >
                  {node}
                </span>
              ))}
            </div>
            <div className="text-xs text-yellow-700 mt-2">
              CPU: N/A | Memory: N/A
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
