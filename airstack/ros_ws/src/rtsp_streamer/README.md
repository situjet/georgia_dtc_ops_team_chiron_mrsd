# rtsp_streamer

Simple ROS 2 (Humble) node that connects to an RTSP source and publishes JPEG `sensor_msgs/CompressedImage` messages at a fixed FPS.

- Default RTSP URL: `rtsp://10.3.1.124:8556/ghadron`
- Default topic: `/image_raw_compressed`
- Default FPS: 5.0

## Run

```
ros2 launch rtsp_streamer rtsp_streamer.launch.py
```

Or directly:

```
ros2 run rtsp_streamer rtsp_streamer_node --ros-args -p rtsp_url:=rtsp://10.3.1.124:8556/ghadron -p topic:=/image_raw_compressed -p fps:=5.0
```

## Parameters

- `rtsp_url` (string): RTSP URL.
- `topic` (string): Image topic to publish.
- `fps` (double): Target frames per second.
- `width` (int): Optional width to request from decoder.
- `height` (int): Optional height to request from decoder.
- `jpeg_quality` (int 1..100): JPEG quality for compression.
- `rtsp_latency_ms` (int): Desired RTSP latency parameter appended to URL if not present.
- `prefer_tcp` (bool): Prefer TCP transport for RTSP.

## Notes

- Uses OpenCV FFmpeg backend for stability. Set `prefer_tcp:=false` to allow UDP.
- If stream provides MJPEG already, OpenCV still re-encodes to JPEG to standardize output.