"use client";

import ROSLIB from "roslib";
import { useCallback, useRef, useState, useEffect } from "react";

interface MotorControlProps {
  ros: ROSLIB.Ros | null;
  isConnected: boolean;
  publishMessage: <T>(topicName: string, message: T) => void;
}

export function MotorControl({ ros, isConnected, publishMessage }: MotorControlProps) {
  const [isDragging, setIsDragging] = useState(false);
  const [knobPosition, setKnobPosition] = useState({ x: 0, y: 0 });
  const [vector, setVector] = useState({ x: 0, y: 0 });
  const [isLocked, setIsLocked] = useState(false);
  const [mode, setMode] = useState<"auto" | "manual">("auto");
  const baseRef = useRef<HTMLDivElement>(null);

  const JOYSTICK_RADIUS = 80;
  const KNOB_RADIUS = 28;

  const sendVector = useCallback((x: number, y: number) => {
    if (!ros || !isConnected) return;
    publishMessage("/motor_vector", { data: JSON.stringify({ x, y }) });
  }, [ros, isConnected, publishMessage]);

  const sendEstop = useCallback(() => {
    if (!ros) return;
    publishMessage("/motor_estop", { data: "estop" });
    setVector({ x: 0, y: 0 });
    setKnobPosition({ x: 0, y: 0 });
    setIsDragging(false);
    setIsLocked(false);
  }, [ros, publishMessage]);

  const toggleMode = useCallback(() => {
    const newMode = mode === "auto" ? "manual" : "auto";
    setMode(newMode);
    publishMessage("/nav_mode", { data: newMode });
    if (newMode === "auto") {
      sendVector(0, 0);
      setKnobPosition({ x: 0, y: 0 });
      setVector({ x: 0, y: 0 });
      setIsLocked(false);
    }
  }, [mode, publishMessage, sendVector]);

  const handleMove = useCallback((clientX: number, clientY: number) => {
    if (!baseRef.current || !isDragging || mode === "auto") return;

    const rect = baseRef.current.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;

    let x = clientX - centerX;
    let y = clientY - centerY;

    const distance = Math.sqrt(x * x + y * y);
    const maxDistance = JOYSTICK_RADIUS - KNOB_RADIUS / 2;
    if (distance > maxDistance) {
      x = (x / distance) * maxDistance;
      y = (y / distance) * maxDistance;
    }

    setKnobPosition({ x, y });

    const normalizedX = Math.round((x / maxDistance) * 100) / 100;
    const normalizedY = Math.round((-y / maxDistance) * 100) / 100;

    if (normalizedX !== vector.x || normalizedY !== vector.y) {
      setVector({ x: normalizedX, y: normalizedY });
      sendVector(normalizedX, normalizedY);
    }
  }, [isDragging, mode, vector, sendVector]);

  const handleRelease = useCallback(() => {
    setIsDragging(false);
    if (!isLocked) {
      setKnobPosition({ x: 0, y: 0 });
      if (vector.x !== 0 || vector.y !== 0) {
        setVector({ x: 0, y: 0 });
        sendVector(0, 0);
      }
    }
  }, [vector, sendVector, isLocked]);

  const handleStart = useCallback((clientX: number, clientY: number) => {
    if (mode === "auto") return;   // ← block joystick in auto mode
    setIsDragging(true);
    if (!baseRef.current) return;

    const rect = baseRef.current.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;

    let x = clientX - centerX;
    let y = clientY - centerY;

    const distance = Math.sqrt(x * x + y * y);
    const maxDistance = JOYSTICK_RADIUS - KNOB_RADIUS / 2;
    if (distance > maxDistance) {
      x = (x / distance) * maxDistance;
      y = (y / distance) * maxDistance;
    }

    setKnobPosition({ x, y });
    const normalizedX = Math.round((x / maxDistance) * 100) / 100;
    const normalizedY = Math.round((-y / maxDistance) * 100) / 100;
    setVector({ x: normalizedX, y: normalizedY });
    sendVector(normalizedX, normalizedY);
  }, [mode, sendVector]);

  useEffect(() => {
    if (!isDragging) return;
    const onMouseMove = (e: MouseEvent) => handleMove(e.clientX, e.clientY);
    const onTouchMove = (e: TouchEvent) => {
      if (e.touches[0]) handleMove(e.touches[0].clientX, e.touches[0].clientY);
    };
    const onEnd = () => handleRelease();
    window.addEventListener("mousemove", onMouseMove);
    window.addEventListener("mouseup", onEnd);
    window.addEventListener("touchmove", onTouchMove);
    window.addEventListener("touchend", onEnd);
    return () => {
      window.removeEventListener("mousemove", onMouseMove);
      window.removeEventListener("mouseup", onEnd);
      window.removeEventListener("touchmove", onTouchMove);
      window.removeEventListener("touchend", onEnd);
    };
  }, [isDragging, handleMove, handleRelease]);

  const isManual = mode === "manual";

  return (
    <div className="p-6 bg-slate-900 rounded-xl border border-slate-700">
      <div className="flex items-center justify-between mb-4">
        <h2 className="text-lg font-semibold text-slate-100">Motor Control</h2>
        {/* AUTO / MANUAL toggle */}
        <button
          onClick={toggleMode}
          className={`px-4 py-1.5 rounded-lg font-bold text-sm transition-all ${
            isManual
              ? "bg-green-600 hover:bg-green-500 text-white"
              : "bg-slate-700 hover:bg-slate-600 text-slate-300"
          }`}
        >
          {isManual ? "MANUAL" : "AUTO"}
        </button>
      </div>

      <div className="flex flex-col items-center gap-5">
        {/* Joystick — dimmed in AUTO mode */}
        <div className={`transition-opacity ${isManual ? "opacity-100" : "opacity-30 pointer-events-none"}`}>
          <div
            ref={baseRef}
            className={`relative rounded-full bg-gradient-to-b from-slate-700 to-slate-800 border-4 cursor-pointer select-none touch-none shadow-inner ${
              isLocked ? "border-amber-500" : "border-slate-600"
            }`}
            style={{ width: JOYSTICK_RADIUS * 2, height: JOYSTICK_RADIUS * 2 }}
            onMouseDown={(e) => handleStart(e.clientX, e.clientY)}
            onTouchStart={(e) => {
              if (e.touches[0]) handleStart(e.touches[0].clientX, e.touches[0].clientY);
            }}
          >
            <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
              <div className="absolute w-full h-px bg-slate-600/50" />
              <div className="absolute h-full w-px bg-slate-600/50" />
            </div>
            <div
              className={`absolute rounded-full shadow-xl border-2 transition-colors ${
                isDragging ? "bg-slate-400 border-slate-300" : "bg-slate-500 border-slate-400"
              }`}
              style={{
                width: KNOB_RADIUS * 2,
                height: KNOB_RADIUS * 2,
                left: "50%",
                top: "50%",
                transform: `translate(calc(-50% + ${knobPosition.x}px), calc(-50% + ${knobPosition.y}px))`,
              }}
            />
          </div>
        </div>

        {/* Vector display */}
        <div className="text-center font-mono text-sm bg-slate-800 px-4 py-2 rounded-lg">
          <span className="text-slate-400">X: </span>
          <span className="text-sky-400">{vector.x.toFixed(2)}</span>
          <span className="text-slate-600 mx-2">|</span>
          <span className="text-slate-400">Y: </span>
          <span className="text-emerald-400">{vector.y.toFixed(2)}</span>
        </div>

        {/* LOCK + E-STOP — always active regardless of mode */}
        <div className="flex gap-2">
          <button
            onClick={() => {
              setIsLocked(!isLocked);
              if (isLocked) {
                setVector({ x: 0, y: 0 });
                setKnobPosition({ x: 0, y: 0 });
                sendVector(0, 0);
              }
            }}
            disabled={!isManual}
            className={`px-4 py-2 font-bold rounded-lg transition-all disabled:opacity-30 ${
              isLocked ? "bg-amber-500 text-white" : "bg-slate-700 text-slate-300 hover:bg-slate-600"
            }`}
          >
            {isLocked ? "LOCKED" : "LOCK"}
          </button>
          <button
            onClick={sendEstop}
            className="px-4 py-2 bg-red-600 hover:bg-red-500 active:bg-red-700 text-white font-bold rounded-lg shadow-lg border-2 border-red-500 transition-all active:scale-95"
          >
            E-STOP
          </button>
        </div>

        {/* Status */}
        <div className="flex items-center gap-3 text-xs">
          <div className={`w-2 h-2 rounded-full ${isConnected ? "bg-emerald-500 animate-pulse" : "bg-red-500"}`} />
          <span className="text-slate-400">{isConnected ? "Connected" : "Disconnected"}</span>
          <span className={`font-semibold ${isManual ? "text-green-400" : "text-slate-500"}`}>
            {isManual ? "● MANUAL" : "○ AUTO"}
          </span>
        </div>
      </div>
    </div>
  );
}