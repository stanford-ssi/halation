"use client";

import { useRos } from "@/hooks/useRos";
import { MotorControl } from "@/components/MotorControl";

function TopicChip({
  topic,
  isSubscribed,
  onToggle,
}: {
  topic: string;
  isSubscribed: boolean;
  onToggle: (topic: string) => void;
}) {
  return (
    <button
      onClick={() => onToggle(topic)}
      className={`px-3 py-1 rounded-full text-sm font-medium transition-colors cursor-pointer ${
        isSubscribed
          ? "bg-blue-500 text-white hover:bg-blue-600"
          : "bg-white text-black"
      }`}
    >
      {topic}
    </button>
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
  const {
    topics,
    loggingTopics,
    logs,
    isConnected,
    ros,
    publishMessage,
    subscribeToTopic,
    toggleTopicLogging,
    clearLogs,
  } = useRos(100);

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
            <TopicChip
              key={topic}
              topic={topic}
              isSubscribed={loggingTopics.includes(topic)}
              onToggle={toggleTopicLogging}
            />
          ))}
        </div>
      </div>

      <MotorControl
        ros={ros}
        isConnected={isConnected}
        publishMessage={publishMessage}
      />

      <div className="mb-4">
        <div className="flex justify-between items-center">
          <h2 className="text-lg font-semibold mb-2">
            Live Logs ({logs.length} messages)
          </h2>

          <button
            onClick={() => clearLogs()}
            className="px-3 py-1 rounded-full text-sm font-medium transition-colors cursor-pointer bg-red-100 text-red-800 hover:bg-red-200"
          >
            Clear Logs
          </button>
        </div>
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
