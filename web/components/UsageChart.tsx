"use client";

import {
  Area,
  AreaChart,
  CartesianGrid,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from "recharts";

interface NodeUsageSnapshot {
  name: string;
  namespace: string;
  pid: number;
  memory_mb: number;
  cpu_percent: number;
  threads: number;
  cmd: string;
}

interface UnmatchedProcessSnapshot {
  pid: number;
  name: string;
  memory_mb: number;
  cpu_percent: number;
  threads: number;
  cmd: string;
}

interface UsageDataPoint {
  timestamp: string;
  time: string;
  totalCpu: number;
  totalMemory: number;
  nodes: NodeUsageSnapshot[];
  unmatched: UnmatchedProcessSnapshot[];
}

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

interface UsageChartProps {
  data: UsageDataPoint[];
  currentCpu: number;
  currentMemory: number;
  topCpuProcesses: (NodeUsage | UnmatchedProcess)[];
  topMemoryProcesses: (NodeUsage | UnmatchedProcess)[];
}

export function UsageChart({
  data,
  currentCpu,
  currentMemory,
  topCpuProcesses,
  topMemoryProcesses,
}: UsageChartProps) {
  const formatYAxis = (value: number, metric: "cpu" | "memory") => {
    if (metric === "cpu") {
      return `${value}%`;
    }
    return `${value} MB`;
  };

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
      {/* CPU Chart */}
      <div className="bg-white border border-gray-200 rounded p-4 relative">
        <div className="flex justify-between items-start mb-3">
          <h3 className="font-semibold text-sm text-gray-700">
            CPU Usage Over Time
          </h3>
          <div className="text-right">
            <div className="text-xs text-blue-600 font-medium">Total CPU</div>
            <div className="text-2xl font-bold text-blue-800">
              {currentCpu.toFixed(1)}%
            </div>
          </div>
        </div>
        <ResponsiveContainer width="100%" height={200}>
          <AreaChart data={data}>
            <defs>
              <linearGradient id="fillCpu" x1="0" y1="0" x2="0" y2="1">
                <stop offset="5%" stopColor="#3B82F6" stopOpacity={0.8} />
                <stop offset="95%" stopColor="#3B82F6" stopOpacity={0.1} />
              </linearGradient>
            </defs>
            <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
            <XAxis
              dataKey="time"
              tick={{ fontSize: 12 }}
              tickLine={false}
              axisLine={{ stroke: "#e5e7eb" }}
            />
            <YAxis
              tickFormatter={(value) => formatYAxis(value, "cpu")}
              tick={{ fontSize: 12 }}
              tickLine={false}
              axisLine={{ stroke: "#e5e7eb" }}
            />
            <Tooltip
              content={({ active, payload }) => {
                if (active && payload && payload.length) {
                  const dataPoint = payload[0].payload as UsageDataPoint;
                  const allProcesses = [
                    ...dataPoint.nodes,
                    ...dataPoint.unmatched,
                  ].sort((a, b) => b.cpu_percent - a.cpu_percent);

                  return (
                    <div className="bg-white border border-gray-200 rounded shadow-lg p-3 max-w-xs">
                      <p className="text-xs text-gray-500 mb-1">
                        {dataPoint.time}
                      </p>
                      <p className="text-sm font-semibold text-blue-600 mb-2">
                        CPU: {payload[0].value}%
                      </p>
                      <div className="border-t border-gray-200 pt-2 space-y-1 max-h-32 overflow-y-auto">
                        {allProcesses.slice(0, 5).map((proc, idx) => (
                          <div
                            key={idx}
                            className="text-xs flex justify-between"
                          >
                            <span className="text-gray-700 truncate">
                              {proc.name}:
                            </span>
                            <span className="text-blue-600 font-medium ml-2">
                              {proc.cpu_percent.toFixed(1)}%
                            </span>
                          </div>
                        ))}
                      </div>
                    </div>
                  );
                }
                return null;
              }}
            />
            <Area
              type="monotone"
              dataKey="totalCpu"
              stroke="#3B82F6"
              strokeWidth={2}
              fill="url(#fillCpu)"
              animationDuration={800}
            />
          </AreaChart>
        </ResponsiveContainer>

        {/* Top CPU Processes */}
        <div className="mt-4 pt-4 border-t border-gray-200">
          <h4 className="font-semibold mb-2 text-xs text-gray-700">
            Top CPU Usage
          </h4>
          <div className="space-y-2">
            {topCpuProcesses.slice(0, 5).map((proc, idx) => (
              <div key={idx} className="flex justify-between items-center">
                <div className="flex-1 min-w-0">
                  <div className="text-xs font-medium truncate">
                    {"name" in proc ? proc.name : proc.name}
                  </div>
                  <div className="text-xs text-gray-500">
                    PID: {proc.pid} | Threads: {proc.threads}
                  </div>
                </div>
                <div className="ml-4 flex items-center gap-2">
                  <div className="w-16 bg-gray-200 rounded-full h-2">
                    <div
                      className="bg-blue-500 h-2 rounded-full"
                      style={{
                        width: `${Math.min(100, proc.cpu_percent)}%`,
                      }}
                    />
                  </div>
                  <span className="text-xs font-semibold w-12 text-right">
                    {proc.cpu_percent.toFixed(1)}%
                  </span>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>

      {/* Memory Chart */}
      <div className="bg-white border border-gray-200 rounded p-4 relative">
        <div className="flex justify-between items-start mb-3">
          <h3 className="font-semibold text-sm text-gray-700">
            Memory Usage Over Time
          </h3>
          <div className="text-right">
            <div className="text-xs text-green-600 font-medium">
              Total Memory
            </div>
            <div className="text-2xl font-bold text-green-800">
              {currentMemory.toFixed(1)} MB
            </div>
          </div>
        </div>
        <ResponsiveContainer width="100%" height={200}>
          <AreaChart data={data}>
            <defs>
              <linearGradient id="fillMemory" x1="0" y1="0" x2="0" y2="1">
                <stop offset="5%" stopColor="#10B981" stopOpacity={0.8} />
                <stop offset="95%" stopColor="#10B981" stopOpacity={0.1} />
              </linearGradient>
            </defs>
            <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
            <XAxis
              dataKey="time"
              tick={{ fontSize: 12 }}
              tickLine={false}
              axisLine={{ stroke: "#e5e7eb" }}
            />
            <YAxis
              tickFormatter={(value) => formatYAxis(value, "memory")}
              tick={{ fontSize: 12 }}
              tickLine={false}
              axisLine={{ stroke: "#e5e7eb" }}
            />
            <Tooltip
              content={({ active, payload }) => {
                if (active && payload && payload.length) {
                  const dataPoint = payload[0].payload as UsageDataPoint;
                  const allProcesses = [
                    ...dataPoint.nodes,
                    ...dataPoint.unmatched,
                  ].sort((a, b) => b.memory_mb - a.memory_mb);

                  return (
                    <div className="bg-white border border-gray-200 rounded shadow-lg p-3 max-w-xs">
                      <p className="text-xs text-gray-500 mb-1">
                        {dataPoint.time}
                      </p>
                      <p className="text-sm font-semibold text-green-600 mb-2">
                        Memory: {payload[0].value} MB
                      </p>
                      <div className="border-t border-gray-200 pt-2 space-y-1 max-h-32 overflow-y-auto">
                        {allProcesses.slice(0, 5).map((proc, idx) => (
                          <div
                            key={idx}
                            className="text-xs flex justify-between"
                          >
                            <span className="text-gray-700 truncate">
                              {proc.name}:
                            </span>
                            <span className="text-green-600 font-medium ml-2">
                              {proc.memory_mb.toFixed(1)} MB
                            </span>
                          </div>
                        ))}
                      </div>
                    </div>
                  );
                }
                return null;
              }}
            />
            <Area
              type="monotone"
              dataKey="totalMemory"
              stroke="#10B981"
              strokeWidth={2}
              fill="url(#fillMemory)"
              animationDuration={800}
            />
          </AreaChart>
        </ResponsiveContainer>

        {/* Top Memory Processes */}
        <div className="mt-4 pt-4 border-t border-gray-200">
          <h4 className="font-semibold mb-2 text-xs text-gray-700">
            Top Memory Usage
          </h4>
          <div className="space-y-2">
            {topMemoryProcesses.slice(0, 5).map((proc, idx) => {
              const totalMemory = topMemoryProcesses.reduce(
                (sum, p) => sum + p.memory_mb,
                0,
              );
              return (
                <div key={idx} className="flex justify-between items-center">
                  <div className="flex-1 min-w-0">
                    <div className="text-xs font-medium truncate">
                      {"name" in proc ? proc.name : proc.name}
                    </div>
                    <div className="text-xs text-gray-500">
                      PID: {proc.pid} | Threads: {proc.threads}
                    </div>
                  </div>
                  <div className="ml-4 flex items-center gap-2">
                    <div className="w-16 bg-gray-200 rounded-full h-2">
                      <div
                        className="bg-green-500 h-2 rounded-full"
                        style={{
                          width: `${Math.min(100, (proc.memory_mb / totalMemory) * 100)}%`,
                        }}
                      />
                    </div>
                    <span className="text-xs font-semibold w-16 text-right">
                      {proc.memory_mb.toFixed(1)} MB
                    </span>
                  </div>
                </div>
              );
            })}
          </div>
        </div>
      </div>
    </div>
  );
}
