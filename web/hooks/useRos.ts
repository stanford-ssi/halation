import ROSLIB from "roslib";
import { useEffect, useRef, useState, useCallback } from "react";

const ROS_BRIDGE_URL = "wss://ssirovers.org/ws";

class RosPubSub {
  private ros: ROSLIB.Ros | null = null;
  private subscribers = new Map<string, ROSLIB.Topic>();
  private callbacks = new Map<string, Set<(msg: unknown) => void>>();
  private loggingTopics = new Set<string>();
  private onMessageCallback?: (topic: string, message: unknown) => void;

  constructor(
    ros: ROSLIB.Ros | null,
    onMessage?: (topic: string, message: unknown) => void,
  ) {
    this.ros = ros;
    this.onMessageCallback = onMessage;
  }

  setRos(ros: ROSLIB.Ros | null) {
    this.ros = ros;
  }

  subscribe(
    topicName: string,
    callback?: (msg: unknown) => void,
    enableLogging: boolean = false,
  ): void {
    if (!this.ros) {
      console.error("ROS not connected");
      return;
    }

    if (callback) {
      if (!this.callbacks.has(topicName)) {
        this.callbacks.set(topicName, new Set());
      }
      this.callbacks.get(topicName)!.add(callback);
    }

    if (enableLogging) {
      this.loggingTopics.add(topicName);
    }

    if (this.subscribers.has(topicName)) {
      console.log(`Added callback to existing subscription: ${topicName}`);
      return;
    }

    this.ros.getTopicType(topicName, (messageType: string) => {
      console.log(`Subscribing to ${topicName} with type ${messageType}`);

      const topic = new ROSLIB.Topic({
        ros: this.ros!,
        name: topicName,
        messageType: messageType,
      });

      topic.subscribe((message) => {
        if (this.loggingTopics.has(topicName)) {
          this.onMessageCallback?.(topicName, message);
        }

        const callbacks = this.callbacks.get(topicName);
        if (callbacks) {
          callbacks.forEach((cb) => cb(message));
        }
      });

      this.subscribers.set(topicName, topic);
      console.log(`Subscribed to topic: ${topicName}`);
    });
  }

  unsubscribe(topicName: string, callback?: (msg: unknown) => void): void {
    if (callback) {
      const callbacks = this.callbacks.get(topicName);
      if (callbacks) {
        callbacks.delete(callback);
        console.log(`Removed callback from topic: ${topicName}`);

        if (callbacks.size === 0) {
          this.unsubscribeCompletely(topicName);
        }
      }
      return;
    }

    this.unsubscribeCompletely(topicName);
  }

  private unsubscribeCompletely(topicName: string): void {
    const topic = this.subscribers.get(topicName);
    if (topic) {
      topic.unsubscribe();
      this.subscribers.delete(topicName);
      this.callbacks.delete(topicName);
      console.log(`Unsubscribed from topic: ${topicName}`);
    }
  }

  publish<T>(topicName: string, message: T): void {
    if (!this.ros) {
      console.error("ROS not connected");
      return;
    }

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: "std_msgs/String",
    });

    const rosMessage = new ROSLIB.Message({
      data: JSON.stringify(message),
    });

    topic.publish(rosMessage);
    console.log(`Published to ${topicName}:`, message);
  }

  isSubscribed(topicName: string): boolean {
    return this.subscribers.has(topicName);
  }

  getSubscribedTopics(): string[] {
    return Array.from(this.subscribers.keys());
  }

  getLoggingTopics(): string[] {
    return Array.from(this.loggingTopics);
  }

  removeLogging(topicName: string): void {
    this.loggingTopics.delete(topicName);
  }

  cleanup(): void {
    this.subscribers.forEach((topic) => {
      topic.unsubscribe();
    });
    this.subscribers.clear();
    this.callbacks.clear();
  }
}

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
  loggingTopics: string[];
  publishMessage: <T>(topicName: string, message: T) => void;
  toggleTopicLogging: (topicName: string) => void;
  subscribeToTopic: (
    topicName: string,
    callback?: (msg: unknown) => void,
    enableLogging?: boolean,
  ) => void;
  unsubscribeFromTopic: (
    topicName: string,
    callback?: (msg: unknown) => void,
  ) => void;
}

export function useRos(maxLogSize: number = 100): UseRosReturn {
  const rosRef = useRef<ROSLIB.Ros | null>(null);
  const [topics, setTopics] = useState<string[]>([]);
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const [subscribedTopics, setSubscribedTopics] = useState<string[]>([]);
  const [loggingTopics, setLoggingTopics] = useState<string[]>([]);
  const pubSubRef = useRef<RosPubSub | null>(null);

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

  useEffect(() => {
    if (!pubSubRef.current) {
      pubSubRef.current = new RosPubSub(rosRef.current, logMessage);
    }
  }, [logMessage]);

  const subscribeToTopic = useCallback(
    (
      topicName: string,
      callback?: (msg: unknown) => void,
      enableLogging: boolean = false,
    ) => {
      if (!pubSubRef.current) return;

      pubSubRef.current.subscribe(topicName, callback, enableLogging);
      setSubscribedTopics(pubSubRef.current.getSubscribedTopics());
      setLoggingTopics(pubSubRef.current.getLoggingTopics());
    },
    [],
  );

  const unsubscribeFromTopic = useCallback(
    (topicName: string, callback?: (msg: unknown) => void) => {
      if (!pubSubRef.current) return;

      pubSubRef.current.unsubscribe(topicName, callback);
      setSubscribedTopics(pubSubRef.current.getSubscribedTopics());
      setLoggingTopics(pubSubRef.current.getLoggingTopics());
    },
    [],
  );

  const publishMessage = useCallback(
    <T>(topicName: string, message: T): void => {
      if (!pubSubRef.current) return;

      pubSubRef.current.publish(topicName, message);
    },
    [],
  );

  const toggleTopicLogging = useCallback((topicName: string): void => {
    if (!pubSubRef.current) return;

    const isCurrentlyLogging = pubSubRef.current
      .getLoggingTopics()
      .includes(topicName);

    if (isCurrentlyLogging) {
      pubSubRef.current.removeLogging(topicName);
    } else {
      pubSubRef.current.subscribe(topicName, undefined, true);
    }
    setSubscribedTopics(pubSubRef.current.getSubscribedTopics());
    setLoggingTopics(pubSubRef.current.getLoggingTopics());
  }, []);

  useEffect(() => {
    if (!rosRef.current) {
      rosRef.current = new ROSLIB.Ros({ url: ROS_BRIDGE_URL });
    }

    const ros = rosRef.current;

    const handleConnection = () => {
      console.log("Connected to rosbridge!");
      setIsConnected(true);

      if (pubSubRef.current) {
        pubSubRef.current.setRos(ros);
      }

      ros.getTopics((result) => {
        setTopics(result.topics);
        console.log("Available topics:", result.topics);

        if (result.topics.includes("/ping")) {
          subscribeToTopic(
            "/ping",
            (msg) => {
              console.log("Ping received:", msg);
            },
            true,
          );
        }
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
  }, [logMessage, subscribeToTopic]);

  useEffect(() => {
    return () => {
      if (pubSubRef.current) {
        pubSubRef.current.cleanup();
      }
    };
  }, []);

  return {
    ros: rosRef.current,
    topics,
    logs,
    clearLogs,
    isConnected,
    subscribedTopics,
    loggingTopics,
    publishMessage,
    toggleTopicLogging,
    subscribeToTopic,
    unsubscribeFromTopic,
  };
}
