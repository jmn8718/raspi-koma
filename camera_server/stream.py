import http.server
import socketserver
import subprocess
import time

PORT = 8080

class StreamHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--frame')
            self.end_headers()

            # Start rpicam-vid to output raw MJPEG to stdout
            # Note: 640x480 is best for the Pi 3B+ CPU
            cmd = [
                "rpicam-vid",
                "-t", "0",
                "--codec", "mjpeg",
                "--width", "640",
                "--height", "480",
                "--framerate", "15",
                "--nopreview",
                "-o", "-"
            ]
            
            # We open the process and read its output buffer
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=0)
            
            try:
                # MJPEG is just a stream of JPEGs starting with 0xff 0xd8 
                # and ending with 0xff 0xd9.
                buffer = b""
                while True:
                    chunk = process.stdout.read(1024)
                    if not chunk:
                        break
                    buffer += chunk
                    
                    # Look for the end of a JPEG frame
                    a = buffer.find(b'\xff\xd8')
                    b = buffer.find(b'\xff\xd9')
                    
                    if a != -1 and b != -1:
                        jpg = buffer[a:b+2]
                        buffer = buffer[b+2:]
                        
                        # Send the MJPEG boundary and the frame
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', len(jpg))
                        self.end_headers()
                        self.wfile.write(jpg)
                        self.wfile.write(b'\r\n')
            except Exception as e:
                print(f"Stream error: {e}")
            finally:
                process.kill()
        else:
            # Simple HTML wrapper
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b"<html><body><h1>Live Feed</h1><img src='/stream.mjpg'></body></html>")

# Use Threading to allow multiple people to view the stream
class ThreadedHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    pass

with ThreadedHTTPServer(("", PORT), StreamHandler) as httpd:
    print(f"Streaming at http://localhost:{PORT}")
    httpd.serve_forever()