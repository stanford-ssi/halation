#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import json
from ros2node.api import get_node_names

class MemoryMonitor(Node):
    def __init__(self):
        super().__init__('memory_monitor')
        self.pub = self.create_publisher(String, '/usage', 10)
        self.create_timer(2.0, self.publish_memory)

        for proc in psutil.process_iter():
            try:
                proc.cpu_percent()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

    def _extract_node_name(self, cmdline):
        if not cmdline:
            return None

        for arg in cmdline:
            if '__node:=' in arg:
                return arg.split(':=')[1]

        for i, arg in enumerate(cmdline):
            if i > 0 and ('ros' in cmdline[i-1].lower() or arg.startswith('/')):
                return arg.split('/')[-1].split()[0]

        return None

    def _get_node_to_pid_mapping(self):
        try:
            rclpy.spin_once(self, timeout_sec=0.1)
            discovered_nodes = get_node_names(node=self, include_hidden_nodes=False)
            node_names = {full_name: (name, namespace) for name, namespace, full_name in discovered_nodes}
        except Exception as e:
            self.get_logger().warn(f'Node discovery failed: {e}')
            node_names = {}

        node_data = {}
        unmatched_processes = []

        for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'memory_info', 'cpu_percent', 'num_threads']):
            try:
                cmdline = proc.info['cmdline']
                if not cmdline:
                    continue

                cmdline_str = ' '.join(cmdline)

                if '--ros-args' in cmdline_str or 'ros2' in cmdline_str.lower():
                    memory_mb = proc.info['memory_info'].rss / 1024 / 1024
                    cpu_percent = proc.info['cpu_percent'] or 0.0
                    num_threads = proc.info['num_threads']

                    node_name = self._extract_node_name(cmdline)

                    if node_name:
                        matched = False
                        for full_name, (name, namespace) in node_names.items():
                            if node_name in full_name or node_name == name:
                                node_data[full_name] = {
                                    'name': name,
                                    'namespace': namespace,
                                    'pid': proc.info['pid'],
                                    'memory_mb': round(memory_mb, 1),
                                    'cpu_percent': round(cpu_percent, 1),
                                    'threads': num_threads,
                                    'cmd': cmdline_str[:50]
                                }
                                matched = True
                                break

                        if not matched:
                            node_data[node_name] = {
                                'name': node_name,
                                'namespace': '/',
                                'pid': proc.info['pid'],
                                'memory_mb': round(memory_mb, 1),
                                'cpu_percent': round(cpu_percent, 1),
                                'threads': num_threads,
                                'cmd': cmdline_str[:50]
                            }
                    else:
                        unmatched_processes.append({
                            'pid': proc.info['pid'],
                            'name': proc.info['name'],
                            'memory_mb': round(memory_mb, 1),
                            'cpu_percent': round(cpu_percent, 1),
                            'threads': num_threads,
                            'cmd': cmdline_str[:50]
                        })

            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass

        return node_data, unmatched_processes

    def publish_memory(self):
        try:
            node_data, unmatched_processes = self._get_node_to_pid_mapping()

            if not node_data and not unmatched_processes:
                msg = String()
                msg.data = json.dumps({"nodes": [], "unmatched": [], "total_cpu": 0, "total_memory_mb": 0})
                self.pub.publish(msg)
                return

            nodes_list = list(node_data.values())
            total_cpu = sum(n['cpu_percent'] for n in nodes_list) + sum(p['cpu_percent'] for p in unmatched_processes)
            total_memory_mb = sum(n['memory_mb'] for n in nodes_list) + sum(p['memory_mb'] for p in unmatched_processes)

            output = {
                "nodes": nodes_list,
                "unmatched": unmatched_processes,
                "total_cpu": round(total_cpu, 1),
                "total_memory_mb": round(total_memory_mb, 1)
            }

            msg = String()
            msg.data = json.dumps(output)
            self.pub.publish(msg)

            if nodes_list:
                top_nodes = sorted(nodes_list, key=lambda x: x['cpu_percent'], reverse=True)[:3]
                node_details = [
                    f"{n['name']}(PID:{n['pid']},CPU:{n['cpu_percent']}%,T:{n['threads']})"
                    for n in top_nodes
                ]
                self.get_logger().info(
                    f'Top Nodes: {", ".join(node_details)} | '
                    f'Total: CPU={total_cpu:.1f}% MEM={total_memory_mb:.1f}MB | '
                    f'All Nodes: {len(nodes_list)}'
                )
        except Exception as e:
            self.get_logger().error(f'Error in publish_memory: {e}')

def main():
    rclpy.init()
    node = MemoryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
