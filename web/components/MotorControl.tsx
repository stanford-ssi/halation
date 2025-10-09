"use client";

import ROSLIB from "roslib";
import { useCallback } from "react";

export type MotorCommand = "forwards" | "stop" | "backwards";

interface MotorControlProps {
  ros: ROSLIB.Ros | null;
  isConnected: boolean;
  publishMessage: <T>(topicName: string, message: T) => void;
}

export function MotorControl({ ros, isConnected, publishMessage }: MotorControlProps) {
  const publishMotorCommand = useCallback(
    (command: MotorCommand): void => {
      if (!ros) {
        console.error("ROS not connected");
        return;
      }

      publishMessage("/motor_command", { command });
    },
    [ros, publishMessage],
  );


  return (
    <div className="mb-6">
      <h2 className="text-lg font-semibold mb-4">Motor Control</h2>
      <div className="flex gap-4">
        <button
          onClick={() => publishMotorCommand("forwards")}
          disabled={!isConnected}
          className="px-3 py-1 bg-gray-200 text-gray-800 rounded-lg font-medium hover:bg-gray-400"
        >
          Forwards
        </button>
        <button
          onClick={() => publishMotorCommand("stop")}
          disabled={!isConnected}
          className="px-3 py-1 bg-gray-200 text-gray-800 rounded-lg font-medium hover:bg-gray-400"
        >
          Stop
        </button>
        <button
          onClick={() => publishMotorCommand("backwards")}
          disabled={!isConnected}
          className="px-3 py-1 bg-gray-200 text-gray-800 rounded-lg font-medium hover:bg-gray-400"
        >
          Backwards
        </button>
      </div>
    </div>
  );
}
