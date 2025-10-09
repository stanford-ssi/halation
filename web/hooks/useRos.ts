import ROSLIB from "roslib";
import { useEffect, useRef, useState, useCallback } from "react";

const ROS_BRIDGE_URL = "wss://ssirovers.org/ws";

export interface LogEntry {
  topic: string;
  message: unknown;
  timestamp: string;
}

export interface UseRosReturn {
  ros: ROSLIB.Ros | null;
  topics: string[];
  logs: LogEntry[];
  clearLogs: () => void;  
  isConnected: boolean;
  subscribedTopics: string[];
  publishMessage: <T>(topicName: string, message: T) => void;
  toggleTopicSubscription: (topicName: string) => void;
}

export function useRos(maxLogSize: number = 100): UseRosReturn {
  const rosRef = useRef<ROSLIB.Ros | null>(null);
  const [topics, setTopics] = useState<string[]>([]);
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const subscribersRef = useRef<Map<string, ROSLIB.Topic>>(new Map());
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const [subscribedTopics, setSubscribedTopics] = useState<string[]>([]);

  const logMessage = useCallback(
    (topic: string, message: unknown): void => {
      const timestamp = new Date().toISOString();
      const logEntry: LogEntry = { topic, message, timestamp };
      setLogs((prev) => [...prev.slice(-maxLogSize + 1), logEntry]);
      console.log(`[${timestamp}] ${topic}:`, message);
    },
    [maxLogSize],
  );

  const clearLogs = useCallback(() => {
    setLogs([]);
  }, []);


  const subscribeToTopic = useCallback(
    (topicName: string, ros: ROSLIB.Ros) => {
      if (subscribersRef.current.has(topicName)) {
        console.log(`Already subscribed to topic: ${topicName}`);
        return;
      }

      if (!ros) return;

      ros.getTopicType(topicName, (messageType: string) => {
        console.log(`Subscribing to ${topicName} with type ${messageType}`);

        const topic = new ROSLIB.Topic({
          ros: ros,
          name: topicName,
          messageType: messageType,
        });

        topic.subscribe((message) => {
          logMessage(topicName, message);
        });

        subscribersRef.current.set(topicName, topic);
        setSubscribedTopics(prev => [...prev, topicName]);
        console.log(`Subscribed to topic: ${topicName}`);
      });
    },
    [logMessage],
  );

  const unsubscribeFromTopic = useCallback((topicName: string) => {
    const topic = subscribersRef.current.get(topicName);
    if (topic) {
      topic.unsubscribe();
      subscribersRef.current.delete(topicName);
      setSubscribedTopics(prev => prev.filter(t => t !== topicName));
      console.log(`Unsubscribed from topic: ${topicName}`);
    }
  }, []);

  const publishMessage = useCallback(
    <T>(topicName: string, message: T): void => {
      if (!rosRef.current) {
        console.error("ROS not connected");
        return;
      }

      const topic = new ROSLIB.Topic({
        ros: rosRef.current,
        name: topicName,
        messageType: "std_msgs/String",
      });

      const rosMessage = new ROSLIB.Message({
        data: JSON.stringify(message),
      });

      topic.publish(rosMessage);
      console.log(`Published to ${topicName}:`, message);
    },
    [],
  );


  const toggleTopicSubscription = useCallback(
    (topicName: string): void => {
      if (!rosRef.current) {
        console.error("ROS not connected");
        return;
      }

      if (subscribersRef.current.has(topicName)) {
        unsubscribeFromTopic(topicName);
      } else {
        subscribeToTopic(topicName, rosRef.current);
      }
    },
    [subscribeToTopic, unsubscribeFromTopic],
  );

  useEffect(() => {
    if (!rosRef.current) {
      rosRef.current = new ROSLIB.Ros({ url: ROS_BRIDGE_URL });
    }

    const ros = rosRef.current;

    const handleConnection = () => {
      console.log("Connected to rosbridge!");
      setIsConnected(true);

      ros.getTopics((result) => {
        setTopics(result.topics);
        console.log("Available topics:", result.topics);

        toggleTopicSubscription("/ping");
      });
    };

    const handleError = (error: unknown) => {
      console.error("Error connecting to rosbridge:", error);
      setIsConnected(false);
    };

    const handleClose = () => {
      console.log("Connection to rosbridge closed.");
      setIsConnected(false);
    };

    const handleTopicChange = () => {
      ros.getTopics((result) => {
        setTopics(result.topics);
      });
    };

    ros.on("connection", handleConnection);
    ros.on("error", handleError);
    ros.on("close", handleClose);
    ros.addListener("topic", handleTopicChange);

    return () => {
      ros.removeListener("connection", handleConnection);
      ros.removeListener("error", handleError);
      ros.removeListener("close", handleClose);
      ros.removeListener("topic", handleTopicChange);
    };
  }, [logMessage, toggleTopicSubscription]);

  useEffect(() => {
    const subscribers = subscribersRef.current;
    return () => {
      subscribers.forEach((topic) => {
        topic.unsubscribe();
      });
      subscribers.clear();
    };
  }, []);

  return {
    ros: rosRef.current,
    topics,
    logs,
    clearLogs,
    isConnected,
    subscribedTopics,
    publishMessage,
    toggleTopicSubscription,
  };
}
