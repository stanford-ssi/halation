"""Stream Mac webcam as individual JPEG snapshots over HTTP for Docker OpenCV."""
import cv2
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import time

CAM_INDEX = 0
PORT = 5555

cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
lock = threading.Lock()
latest_jpg = None


def capture_loop():
    global latest_jpg
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        _, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with lock:
            latest_jpg = jpg.tobytes()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/frame':
            # Single JPEG snapshot - used by Docker vision node
            with lock:
                jpg = latest_jpg
            if jpg is None:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg)
        else:
            # MJPEG stream for browser preview
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                with lock:
                    jpg = latest_jpg
                if jpg is None:
                    time.sleep(0.03)
                    continue
                try:
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(jpg)
                    self.wfile.write(b'\r\n')
                    time.sleep(0.03)
                except BrokenPipeError:
                    break

    def log_message(self, format, *args):
        pass


class ThreadedHTTPServer(HTTPServer):
    """Handle each request in a new thread."""
    def process_request(self, request, client_address):
        t = threading.Thread(target=self.finish_request, args=(request, client_address), daemon=True)
        t.start()


t = threading.Thread(target=capture_loop, daemon=True)
t.start()

print(f"Camera opened: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
print(f"Streaming on http://0.0.0.0:{PORT}/")
print(f"  Browser preview: http://localhost:{PORT}/")
print(f"  Single frame:    http://localhost:{PORT}/frame")
ThreadedHTTPServer(('0.0.0.0', PORT), Handler).serve_forever()
