#!/bin/bash

# Enhanced RTSP Test Server with CORS Proxy
# This script sets up RTSP server and a CORS proxy for Foxglove

echo "Setting up enhanced RTSP streaming environment for Foxglove..."

# Check if rtsp-simple-server is available
if [ ! -f "./rtsp-simple-server" ]; then
    echo "Error: rtsp-simple-server not found in current directory"
    exit 1
fi

# Start the RTSP server in the background
echo "Starting RTSP server on port 8554..."
./rtsp-simple-server &
SERVER_PID=$!

# Wait for server to start
sleep 2

echo "RTSP server started with PID: $SERVER_PID"
echo "Available endpoints:"
echo "  - RTSP: rtsp://localhost:8554/stream"
echo "  - HLS: http://localhost:8888/stream/index.m3u8"
echo "  - WebRTC: http://localhost:8889/stream/whep"
echo ""

# Start streaming test content
echo "Streaming test content to RTSP server..."
ffmpeg -f lavfi -i testsrc=size=640x480:rate=30 -f lavfi -i sine=frequency=1000 \
    -c:v libx264 -preset ultrafast -tune zerolatency -c:a aac -ar 44100 \
    -f rtsp rtsp://localhost:8554/stream &
FFMPEG_PID=$!

echo "FFmpeg streaming with PID: $FFMPEG_PID"
echo ""

# Start a simple CORS proxy using Python
echo "Starting CORS proxy on port 8890 for Foxglove compatibility..."
python3 -c "
import http.server
import socketserver
import urllib.request
from urllib.parse import urlparse
import sys

class CORSProxyHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        # Extract the target URL from the path
        target_url = self.path[1:]  # Remove leading slash
        if not target_url.startswith('http'):
            target_url = 'http://localhost:8888' + self.path
        
        try:
            # Fetch the content from the target URL
            with urllib.request.urlopen(target_url) as response:
                content = response.read()
                content_type = response.headers.get('Content-Type', 'application/octet-stream')
            
            # Send response with CORS headers
            self.send_response(200)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', '*')
            self.send_header('Content-Type', content_type)
            self.end_headers()
            self.wfile.write(content)
            
        except Exception as e:
            self.send_response(500)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(f'Error: {str(e)}'.encode())
    
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', '*')
        self.end_headers()

with socketserver.TCPServer(('', 8890), CORSProxyHandler) as httpd:
    print('CORS Proxy server running on port 8890')
    httpd.serve_forever()
" &
PROXY_PID=$!

echo "CORS proxy started with PID: $PROXY_PID"
echo ""
echo "=== Foxglove Configuration ==="
echo "Use these URLs in the Foxglove RTSP extension:"
echo ""
echo "1. HLS (Recommended): http://localhost:8890/stream/index.m3u8"
echo "2. Direct HLS: http://localhost:8888/stream/index.m3u8"
echo "3. RTSP (Limited): rtsp://localhost:8554/stream"
echo ""
echo "Note: Use HLS with CORS proxy (option 1) for best Foxglove compatibility"
echo ""
echo "Press Ctrl+C to stop all services"

# Cleanup function
cleanup() {
    echo ""
    echo "Stopping services..."
    kill $FFMPEG_PID 2>/dev/null
    kill $SERVER_PID 2>/dev/null
    kill $PROXY_PID 2>/dev/null
    exit 0
}

# Set trap for cleanup
trap cleanup INT TERM

# Wait for interrupt
wait