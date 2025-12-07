"use client";

import ROSLIB from "roslib";
import { useCallback } from "react";

export type MotorCommand = "forwards" | "backwards" | "left" | "right" | "stop";

interface MotorControlProps {
  ros: ROSLIB.Ros | null;
  isConnected: boolean;
  publishMessage: <T>(topicName: string, message: T) => void;
}

export function MotorControl({
  ros,
  isConnected,
  publishMessage,
}: MotorControlProps) {
  const sendCommand = useCallback(
    (command: MotorCommand): void => {
      if (!ros) {
        console.error("ROS not connected");
        return;
      }
      publishMessage("/motor_command", { data: command });
    },
    [ros, publishMessage],
  );

  const buttonBase =
    "px-4 py-3 rounded-lg font-medium transition-all disabled:opacity-50 disabled:cursor-not-allowed";
  const directionButton = `${buttonBase} bg-slate-700 text-white hover:bg-slate-600 active:bg-slate-800`;
  const stopButton = `${buttonBase} bg-red-600 text-white hover:bg-red-500 active:bg-red-700`;

  return (
    <div className="p-6 bg-slate-900 rounded-xl border border-slate-700">
      <h2 className="text-lg font-semibold mb-6 text-slate-100">
        Motor Control
      </h2>

      <div className="flex flex-col items-center gap-3">
        {/* Forward */}
        <button
          onClick={() => sendCommand("forwards")}
          disabled={!isConnected}
          className={directionButton}
        >
          ▲ Forward
        </button>

        {/* Left / Stop / Right */}
        <div className="flex gap-3">
          <button
            onClick={() => sendCommand("left")}
            disabled={!isConnected}
            className={directionButton}
          >
            ◀ Left
          </button>
          <button
            onClick={() => sendCommand("stop")}
            disabled={!isConnected}
            className={stopButton}
          >
            ■ Stop
          </button>
          <button
            onClick={() => sendCommand("right")}
            disabled={!isConnected}
            className={directionButton}
          >
            Right ▶
          </button>
        </div>

        {/* Backward */}
        <button
          onClick={() => sendCommand("backwards")}
          disabled={!isConnected}
          className={directionButton}
        >
          ▼ Backward
        </button>
      </div>
    </div>
  );
}
