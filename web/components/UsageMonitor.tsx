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

export function UsageMonitor({ ros, isConnected, topics }: UsageMonitorProps) {
  const [usageData, setUsageData] = useState<UsageData | null>(null);
  const [allNodes, setAllNodes] = useState<string[]>([]);
  const [usageHistory, setUsageHistory] = useState<UsageHistoryPoint[]>([]);

  useEffect(() => {
    if (!isConnected || !ros) return;

    console.log("Subscribing to /usage topic...");

    const usageTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/usage",
      messageType: "std_msgs/String",
    });

    usageTopic.subscribe((msg: unknown) => {
      console.log("Received /usage message:", msg);
      try {
        const data = msg as { data: string };
        console.log("Raw data:", data.data);
        const parsed = JSON.parse(data.data) as UsageData;
        console.log("Parsed usage data:", parsed);
        setUsageData(parsed);

        // Add to history for charting (keep last 30 data points)
        const now = new Date();
        const timeStr = now.toLocaleTimeString("en-US", {
          hour: "numeric",
          minute: "2-digit",
          second: "2-digit",
        });

        setUsageHistory((prev) => {
          const newPoint: UsageHistoryPoint = {
            timestamp: now.toISOString(),
            time: timeStr,
            totalCpu: parsed.total_cpu,
            totalMemory: parsed.total_memory_mb,
            nodes: parsed.nodes,
            unmatched: parsed.unmatched,
          };
          const updated = [...prev, newPoint];
          // Keep last 30 points (1 minute of data at 2s intervals)
          return updated.slice(-30);
        });
      } catch (error) {
        console.error("Failed to parse usage data:", error, msg);
      }
    });

    return () => {
      usageTopic.unsubscribe();
      console.log("Unsubscribed from /usage topic");
    };
  }, [isConnected, ros]);

  // Discover nodes from available topics
  useEffect(() => {
    if (!topics || topics.length === 0) return;

    // Extract unique node names from topics
    // ROS topics typically follow pattern: /node_name/topic_name or /topic_name
    const nodeSet = new Set<string>();

    topics.forEach((topic) => {
      const parts = topic.split("/").filter((p) => p.length > 0);
      if (parts.length > 1) {
        // Topic like /node_name/topic_name
        nodeSet.add(parts[0]);
      }
    });

    setAllNodes(Array.from(nodeSet));
  }, [topics]);

  if (!isConnected) {
    return (
      <div className="mb-6 p-4 bg-gray-100 rounded">
        <h2 className="text-lg font-semibold mb-2">System Usage</h2>
        <div className="text-gray-500">Not connected to ROS</div>
      </div>
    );
  }

  if (!usageData) {
    return (
      <div className="mb-6 p-4 bg-gray-100 rounded">
        <h2 className="text-lg font-semibold mb-2">System Usage</h2>
        <div className="text-gray-500">Waiting for usage data...</div>
      </div>
    );
  }

  const trackedNodeNames = new Set(usageData.nodes.map((n) => n.name));
  const missingNodes = allNodes.filter((node) => !trackedNodeNames.has(node));

  const allProcesses = [
    ...usageData.nodes.map((n) => ({ ...n, type: "node" as const })),
    ...usageData.unmatched.map((u) => ({ ...u, type: "unmatched" as const })),
  ];

  const sortedByCpu = [...allProcesses].sort(
    (a, b) => b.cpu_percent - a.cpu_percent,
  );
  const sortedByMemory = [...allProcesses].sort(
    (a, b) => b.memory_mb - a.memory_mb,
  );

  return (
    <div className="mb-6">
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
