# RTSP Stream Viewer (Foxglove Panel)

A Foxglove panel that displays RTSP camera feeds by playing their HLS rendition using hls.js. This avoids the browser error “Auto-play failed: Failed to load because no supported source was found” by serving a browser-compatible `.m3u8` stream instead of raw RTSP.

## Quick start

1. Install deps and build the extension:
   ```bash
   cd operator/rtsp-stream-viewer
   npm install
   npm run build
   npm run local-install
   ```

2. Start the bundled RTSP/HLS test server:
   ```bash
   ./test_rtsp_server.sh
   ```

3. In Foxglove Studio (desktop), add the panel “RTSP Stream Viewer” and Connect to:
   - `http://localhost:8888/stream/index.m3u8` (HLS)
   - Note: If you paste an `rtsp://...` URL, the panel auto-converts it to HLS using the same host and path.

Foxglove Bridge note: Your ROS/Foxglove bridge listening on `localhost:8765` is unrelated to the video URL; this panel plays a direct HTTP/HLS URL alongside ROS messages.

## How it works

- Browsers can’t play `rtsp://` directly. The panel detects RTSP URLs and converts them to the HLS endpoint provided by `rtsp-simple-server`:
  - `rtsp://HOST[:PORT]/PATH` → `http://HOST:8888/PATH/index.m3u8`
- It uses `hls.js` for playback in Chromium-based browsers; Safari can play HLS natively.
- The `<video>` element is muted and `playsInline`, so autoplay typically succeeds. If the browser still blocks autoplay, press Play once.

## Supported URLs

- HLS: `http://host:8888/<path>/index.m3u8`
- MP4 / WebM / MJPEG direct links
- RTSP input is accepted and auto-mapped to HLS

## Troubleshooting

- “No supported source”: Ensure you’re using the HLS URL (.m3u8). If you provide RTSP, the panel maps it to `http://host:8888/.../index.m3u8` but the HLS server must be running.
- CORS: The `rtsp-simple-server.yml` in this project sets `hlsAllowOrigin: '*'`. If you change host/ports, ensure CORS is still allowed for Foxglove to fetch segments.
- Network: Confirm `localhost:8888` (HLS) and `localhost:8554` (RTSP) are reachable. The test script starts both.
- Codecs: HLS segments must contain browser-decodable codecs (H.264/AAC are safe). If VLC plays but the browser doesn’t, check codec compatibility.
- Autoplay: The panel sets `muted` and tries autoplay; if the browser blocks it, click Play. You can also interact with the page (a click) before connecting.

## Useful scripts

```bash
# Build and install the panel
npm install
npm run build
npm run local-install

# Start test servers (RTSP + HLS)
./test_rtsp_server.sh

# Optional: launch a tiny Foxglove test server
./foxglove_test_server.sh
```

## Notes

- This panel doesn’t depend on ROS topics; it’s a pure video player panel in Foxglove.
- For lower latency, consider enabling WebRTC in `rtsp-simple-server` and adding a WebRTC player; HLS is simple and robust but not ultra-low-latency by default.

## License

MIT