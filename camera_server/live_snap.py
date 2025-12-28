import http.server
import socketserver
import subprocess
import os

PORT = 8080

class SnapHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        # Only trigger the camera when the main page is requested
        if self.path == "/" or self.path == "/index.html":
            print("Refresh detected! Taking a new photo...")
            
            # 1. Take the photo using the tool you built
            # We use --immediate and --nopreview for speed on the Pi 3B+
            try:
                subprocess.run([
                    "rpicam-still", 
                    "-o", "latest.jpg", 
                    "--immediate", 
                    "--nopreview",
                    "--width", "1280", 
                    "--height", "720"
                ], check=True)
            except Exception as e:
                print(f"Camera error: {e}")

            # 2. Serve a simple HTML page that shows the image
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            
            html = f"""
            <html>
                <head>
                    <title>Pi Snap</title>
                    <style>
                        body {{ font-family: sans-serif; text-align: center; background: #1a1a1a; color: white; padding: 20px; }}
                        img {{ max-width: 90%; border: 5px solid #333; border-radius: 10px; box-shadow: 0 4px 15px rgba(0,0,0,0.5); }}
                        .btn {{ background: #007bff; color: white; padding: 10px 20px; text-decoration: none; border-radius: 5px; }}
                    </style>
                </head>
                <body>
                    <h1>Pi 3B+ Live Snap</h1>
                    <p>Last capture: latest.jpg</p>
                    <img src="/latest.jpg?t={os.urandom(8).hex()}">
                    <br><br>
                    <a href="/" class="btn">Take New Photo (Refresh)</a>
                </body>
            </html>
            """
            self.wfile.write(html.encode())
        else:
            # Serve the actual image file when the browser asks for /latest.jpg
            super().do_GET()

with socketserver.TCPServer(("", PORT), SnapHandler) as httpd:
    print(f"Server started at http://localhost:{PORT}")
    httpd.serve_forever()