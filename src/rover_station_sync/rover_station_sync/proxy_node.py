from fastapi import FastAPI, WebSocket
from fastapi_proxy_lib.fastapi.app import reverse_http_app
from rclpy.node import Node
import rclpy
import threading
import uvicorn

class FastAPIProxyNode(Node):
    def __init__(self):
        super().__init__('fastapi_proxy_node')
        self.port = 6000
        self.rosbridge_port = 9090

        self.get_logger().info(f"Station Sync Proxy Node @port={self.port}, @rosbridge_port={self.rosbridge_port}")

        app = FastAPI()
        ws_proxy = reverse_http_app(base_url=f"ws://localhost:{self.rosbridge_port}/") # rosbridge_server

        @app.get("/")
        async def root():
            return "SSI Antarctic Rover API ;)"

        @app.websocket("/ws")
        async def ws(websocket: WebSocket):
            await ws_proxy.proxy(websocket)

        # run FastAPI in a separate thread so rclpy spin() is not blocked
        thread = threading.Thread(target=lambda: uvicorn.run(app, host="0.0.0.0", port=self.port, log_level="info"))
        thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = FastAPIProxyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
