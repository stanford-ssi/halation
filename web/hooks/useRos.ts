import ROSLIB from "roslib";
import { useEffect, useRef } from "react";

const ROS_BRIDGE_URL = "wss://ssirovers.org/ws";

export function useRos() {
  const rosRef = useRef<ROSLIB.Ros>(new ROSLIB.Ros({ url: ROS_BRIDGE_URL }));

  useEffect(() => {
    rosRef.current.on("connection", () => {
      console.log("Connected to rosbridge!");
      rosRef.current.getTopics((result) => {
        console.log("Topics: ", result.topics);
      });
    });

    rosRef.current.on("close", () => {
      console.log("Connection to rosbridge closed.");
    });
  }, [rosRef]);

  return rosRef.current;
}
