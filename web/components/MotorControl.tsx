"use client";

import ROSLIB from "roslib";
import { useCallback } from "react";

// Type definitions
export type MotorCommand = "forwards" | "stop" | "backwards";

export interface MotorControlMessage {
  command: MotorCommand;
}

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

      const motorMessage: MotorControlMessage = { command };
      publishMessage("/motor_command", motorMessage);
    },
    [ros, publishMessage],
  );

  const handleMotorCommand = (command: MotorCommand): void => {
    publishMotorCommand(command);
  };

  return (
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
  );
}
