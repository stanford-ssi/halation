"use client";

import { useRos, MotorCommand } from "@/hooks/useRos";

function TopicChip({ topic }: { topic: string }) {
  return (
    <div className="px-3 py-1 bg-green-100 text-green-800 rounded-full text-sm font-medium">
      {topic}
    </div>
  );
}

function LogEntry({
  log,
}: {
  log: { topic: string; message: unknown; timestamp: string };
}) {
  return (
    <div className="border-b border-gray-200 p-2 text-sm">
      <div className="font-mono text-xs text-gray-500">{log.timestamp}</div>
      <span className="font-bold text-blue-600">
        {log.topic}
        <span className="text-gray-800 font-normal">
          : {JSON.stringify(log.message, null, 2)}
        </span>
      </span>
    </div>
  );
}

export default function Home() {
  const { topics, logs, isConnected, publishMotorCommand } = useRos(100);

  const handleMotorCommand = (command: MotorCommand): void => {
    publishMotorCommand(command);
  };

  return (
    <div className="p-4">
      <h1 className="text-2xl font-bold mb-4">
        ROS Topic Monitor -{" "}
        <span className={isConnected ? "text-green-500" : "text-red-500"}>
          {isConnected ? "Connected" : "Disconnected"}
        </span>
      </h1>

      <div className="mb-6">
        <h2 className="text-lg font-semibold mb-2">
          Available Topics ({topics.length})
        </h2>
        <div className="flex flex-wrap gap-2">
          {topics.map((topic) => (
            <TopicChip key={topic} topic={topic} />
          ))}
        </div>
      </div>

      <div className="mb-6">
        <h2 className="text-lg font-semibold mb-4">Motor Control</h2>
        <div className="flex gap-4">
          <button
            onClick={() => handleMotorCommand("forwards")}
            disabled={!isConnected}
            className="px-6 py-3 bg-gray-200 text-gray-800 rounded-lg font-medium hover:bg-gray-300 disabled:bg-gray-100 disabled:text-gray-400 disabled:cursor-not-allowed transition-colors"
          >
            Forwards
          </button>
          <button
            onClick={() => handleMotorCommand("stop")}
            disabled={!isConnected}
            className="px-6 py-3 bg-gray-200 text-gray-800 rounded-lg font-medium hover:bg-gray-300 disabled:bg-gray-100 disabled:text-gray-400 disabled:cursor-not-allowed transition-colors"
          >
            Stop
          </button>
          <button
            onClick={() => handleMotorCommand("backwards")}
            disabled={!isConnected}
            className="px-6 py-3 bg-gray-200 text-gray-800 rounded-lg font-medium hover:bg-gray-300 disabled:bg-gray-100 disabled:text-gray-400 disabled:cursor-not-allowed transition-colors"
          >
            Backwards
          </button>
        </div>
      </div>

      <div className="mb-4">
        <h2 className="text-lg font-semibold mb-2">
          Live Logs ({logs.length} messages)
        </h2>
        <div className="bg-gray-100 rounded p-4 max-h-96 overflow-y-auto">
          {logs.length === 0 ? (
            <div className="text-gray-500">
              No messages yet. Waiting for data from topics...
            </div>
          ) : (
            logs.map((log, index) => (
              <LogEntry key={`${log.timestamp}-${index}`} log={log} />
            ))
          )}
        </div>
      </div>
    </div>
  );
}
