#!/bin/bash

# Test RTSP Server Setup
# This script sets up both an RTSP server and streams test content to it

echo "Setting up test RTSP streaming environment..."

# Check if rtsp-simple-server is available
if [ ! -f "./rtsp-simple-server" ]; then
    echo "Error: rtsp-simple-server not found in current directory"
    exit 1
fi

# Start the RTSP server in the background
echo "Starting RTSP server on port 8554..."
./rtsp-simple-server &
SERVER_PID=$!

# Wait a moment for the server to start
sleep 2

echo "RTSP server started with PID: $SERVER_PID"
echo "Server is running on rtsp://localhost:8554/stream"
echo ""
echo "Now streaming test content to the server..."

# Create a test pattern and stream it to the RTSP server
# This will push a test stream to the server
ffmpeg -f lavfi -i testsrc=size=640x480:rate=30 -f lavfi -i sine=frequency=1000 \
    -c:v libx264 -preset ultrafast -tune zerolatency -c:a aac -ar 44100 \
    -f rtsp rtsp://localhost:8554/stream &
FFMPEG_PID=$!

echo "FFmpeg streaming with PID: $FFMPEG_PID"
echo ""
echo "Test RTSP stream is now available at: rtsp://localhost:8554/stream"
echo "You can now test the Foxglove extension with this URL"
echo ""
echo "Press Ctrl+C to stop both the server and stream"

# Wait for interrupt signal
trap 'echo "Stopping services..."; kill $FFMPEG_PID $SERVER_PID; exit' INT
wait