import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState, useRef } from "react";
import Hls, { Events as HlsEvents, ErrorTypes as HlsErrorTypes, type ErrorData as HlsErrorData } from "hls.js";
import { createRoot } from "react-dom/client";

function RTSPStreamPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [streamUrl, setStreamUrl] = useState<string>("http://localhost:8888/stream/index.m3u8");
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const videoRef = useRef<HTMLVideoElement>(null);
  const hlsRef = useRef<Hls | null>(null);
  // Setup render handler
  useLayoutEffect(() => {
    context.onRender = (_, done) => {
      setRenderDone(() => done);
    };
    
    // We don't need to watch any specific ROS topics for RTSP streaming
    // But we still need a render handler to keep the panel active
  }, [context]);

  // invoke the done callback once the render is complete
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Convert an RTSP URL to the equivalent HLS URL served by rtsp-simple-server
  const rtspToHls = (url: string): string => {
    try {
      const u = new URL(url);
      // Use same hostname, default HLS port 8888, and same path with /index.m3u8 suffix
      const host = u.hostname || "localhost";
      const path = u.pathname.replace(/^\/+/, "");
      const hlsPath = path.length > 0 ? `${path}/index.m3u8` : "index.m3u8";
      return `http://${host}:8888/${hlsPath}`;
    } catch {
      return url;
    }
  };

  // Simple stream connection (supports HLS via hls.js and native formats)
  const connectToStream = async () => {
    try {
      setError(null);
      setIsConnected(false);

      if (!videoRef.current) {
        throw new Error("Video element not available");
      }

      const video = videoRef.current;
      // Cleanup any existing HLS instance before re-connecting
      if (hlsRef.current) {
        hlsRef.current.destroy();
        hlsRef.current = null;
      }
      
      // Normalize URL: if RTSP, translate to HLS
      const normalizedUrl = /^rtsp(s)?:\/\//i.test(streamUrl) ? rtspToHls(streamUrl) : streamUrl;
      console.log(`Connecting to: ${normalizedUrl}`);
      
      // Reset the element before attaching a new source
      video.pause();
      video.removeAttribute("src");
      video.load();
      
      video.onloadstart = () => {
        console.log("Starting to load stream...");
      };
      
      video.oncanplay = () => {
        setIsConnected(true);
        console.log("Stream ready to play");
      };
      
      video.onloadedmetadata = () => {
        console.log(`Video loaded: ${video.videoWidth}x${video.videoHeight}`);
      };
      
      video.onerror = () => {
        let errorMsg = "Failed to load stream";
        
        if (video.error) {
          switch (video.error.code) {
            case video.error.MEDIA_ERR_ABORTED:
              errorMsg = "Stream loading aborted";
              break;
            case video.error.MEDIA_ERR_NETWORK:
              errorMsg = "Network error - check URL and connectivity";
              break;
            case video.error.MEDIA_ERR_DECODE:
              errorMsg = "Cannot decode stream - unsupported format";
              break;
            case video.error.MEDIA_ERR_SRC_NOT_SUPPORTED:
              errorMsg = "Stream format not supported by browser";
              break;
            default:
              errorMsg = `Media error code: ${video.error.code}`;
          }
        }
        
        setError(errorMsg);
        setIsConnected(false);
      };

      const isHls = /\.m3u8(\?.*)?$/i.test(normalizedUrl);

      if (isHls) {
        if (Hls.isSupported()) {
          const hls = new Hls({
            // Favor low latency if configured that way server-side
            lowLatencyMode: true,
            // Allow cross-origin fetches (server should send CORS headers)
            // debug: true,
          });
          hlsRef.current = hls;
          hls.on(HlsEvents.ERROR, (_evt: string, data: HlsErrorData) => {
            if (data?.fatal) {
              switch (data.type) {
                case HlsErrorTypes.NETWORK_ERROR:
                  setError("HLS network error - check server and CORS");
                  hls.startLoad();
                  break;
                case HlsErrorTypes.MEDIA_ERROR:
                  setError("HLS media error - attempting recovery");
                  hls.recoverMediaError();
                  break;
                default:
                  setError("Fatal HLS error");
                  hls.destroy();
                  hlsRef.current = null;
              }
              setIsConnected(false);
            }
          });
          hls.loadSource(normalizedUrl);
          hls.attachMedia(video);
        } else if (video.canPlayType("application/vnd.apple.mpegurl")) {
          // Safari (and some environments) can play HLS natively
          video.src = normalizedUrl;
          video.load();
        } else {
          setError("HLS not supported in this environment");
          setIsConnected(false);
          return;
        }
      } else {
        // Non-HLS: try direct
        video.src = normalizedUrl;
        video.load();
      }

      // Try autoplay immediately; if it fails, try again on canplay
      const tryPlay = async () => {
        try {
          await video.play();
          setError(null);
        } catch (err) {
          console.warn("Auto-play failed:", err);
          setError(
            `Auto-play failed: ${err instanceof Error ? err.message : String(
              err,
            )}. Click play button manually.`,
          );
        }
      };
      void tryPlay();

      const handleCanPlay = () => void tryPlay();
      video.addEventListener("canplay", handleCanPlay, { once: true });
      
    } catch (err) {
      console.error("Connection error:", err);
      setError(`Connection error: ${err instanceof Error ? err.message : String(err)}`);
      setIsConnected(false);
    }
  };

  const disconnectFromStream = () => {
    if (videoRef.current) {
      videoRef.current.pause();
      videoRef.current.removeAttribute("src");
      videoRef.current.load();
    }
    if (hlsRef.current) {
      hlsRef.current.destroy();
      hlsRef.current = null;
    }
    setIsConnected(false);
    setError(null);
  };

  return (
    <div style={{ padding: "1rem", height: "100%", display: "flex", flexDirection: "column" }}>
      <h2>Video Stream Viewer</h2>
      
      {/* Stream URL Input */}
      <div style={{ marginBottom: "1rem" }}>
        <label htmlFor="stream-url" style={{ display: "block", marginBottom: "0.5rem" }}>
          Stream URL:
        </label>
        <div style={{ display: "flex", gap: "0.5rem" }}>
          <input
            id="stream-url"
            type="text"
            value={streamUrl}
            onChange={(e) => setStreamUrl(e.target.value)}
            placeholder="http://localhost:8888/stream/index.m3u8"
            style={{
              flex: 1,
              padding: "0.5rem",
              border: "1px solid #ccc",
              borderRadius: "4px",
            }}
            disabled={isConnected}
          />
          <button
            onClick={isConnected ? disconnectFromStream : connectToStream}
            style={{
              padding: "0.5rem 1rem",
              backgroundColor: isConnected ? "#dc3545" : "#007bff",
              color: "white",
              border: "none",
              borderRadius: "4px",
              cursor: "pointer",
            }}
          >
            {isConnected ? "Disconnect" : "Connect"}
          </button>
        </div>
        <div style={{ fontSize: "0.8rem", color: "#666", marginTop: "0.25rem" }}>
          Supports: HLS (.m3u8 via hls.js), MP4, WebM, MJPEG streams. If you have an RTSP URL,
          it will be auto-converted to HLS as <code>http://host:8888/your_path/index.m3u8</code>.
        </div>
      </div>

      {/* Status Indicator */}
      <div style={{ marginBottom: "1rem" }}>
        <span
          style={{
            display: "inline-block",
            padding: "0.25rem 0.5rem",
            borderRadius: "4px",
            fontSize: "0.8rem",
            backgroundColor: isConnected ? "#28a745" : error ? "#dc3545" : "#6c757d",
            color: "white",
          }}
        >
          {isConnected ? "Connected" : error ? "Error" : "Disconnected"}
        </span>
        {error && (
          <div style={{ color: "#dc3545", fontSize: "0.9rem", marginTop: "0.25rem" }}>
            {error}
          </div>
        )}
      </div>

      {/* Video Stream Display */}
      <div style={{ flex: 1, minHeight: "300px", backgroundColor: "#000", borderRadius: "4px", position: "relative" }}>
        <video
          ref={videoRef}
          style={{
            width: "100%",
            height: "100%",
            objectFit: "contain",
          }}
          controls
          muted
          playsInline
          crossOrigin="anonymous"
        />
        {!isConnected && !error && (
          <div
            style={{
              position: "absolute",
              top: "50%",
              left: "50%",
              transform: "translate(-50%, -50%)",
              color: "white",
              textAlign: "center",
            }}
          >
            <p>Enter RTSP URL and click Connect to view stream</p>
            <p style={{ fontSize: "0.8rem", opacity: 0.7 }}>
              Note: Direct RTSP playback may not work in all browsers.<br />
              For production use, consider using WebRTC or HLS/DASH conversion.
            </p>
          </div>
        )}
      </div>

      {/* Stream Information */}
      {isConnected && videoRef.current && (
        <div style={{ marginTop: "1rem", fontSize: "0.9rem", color: "#666" }}>
          <div>Resolution: {videoRef.current.videoWidth}x{videoRef.current.videoHeight}</div>
          <div>URL: {streamUrl}</div>
          <div>Status: Connected and playing</div>
        </div>
      )}
    </div>
  );
}

export function initRTSPStreamPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<RTSPStreamPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
