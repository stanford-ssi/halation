from fastapi import FastAPI, WebSocket
from fastapi_proxy_lib.core.websocket import ReverseWebSocketProxy
from httpx import AsyncClient
from rclpy.node import Node
import rclpy
import threading
import uvicorn

class FastAPIProxyNode(Node):
    def __init__(self):
        super().__init__('fastapi_proxy_node')
        self.port = 6067
        self.rosbridge_port = 9095

        self.get_logger().info(f"Station Sync Proxy Node @port={self.port}, @rosbridge_port={self.rosbridge_port}")
        self.get_logger().info("[PLACEHOLDER] Log connection to rosbridge status here")

        app = FastAPI()

        self.ws_proxy = ReverseWebSocketProxy(
            client=AsyncClient(),
            base_url=f"ws://localhost:{self.rosbridge_port}/"
        )

        @app.get("/")
        async def root():
            return "SSI Antarctic Rover API ;)"

        @app.websocket("/ws")
        async def ws(websocket: WebSocket):
            await self.ws_proxy.proxy(websocket=websocket)


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
