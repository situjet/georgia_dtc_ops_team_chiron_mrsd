import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState, useRef } from "react";
import { createRoot } from "react-dom/client";

function RTSPStreamPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [rtspUrl, setRtspUrl] = useState<string>("rtsp://localhost:8554/stream");
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const videoRef = useRef<HTMLVideoElement>(null);
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

  // Handle RTSP stream connection
  const connectToRTSP = async () => {
    try {
      setError(null);
      setIsConnected(false);

      if (!videoRef.current) {
        throw new Error("Video element not available");
      }

      // For RTSP streams in browsers, we need to use WebRTC or HLS/DASH conversion
      // Since direct RTSP is not supported in browsers, we'll simulate with a placeholder
      // In a real implementation, you'd use a WebRTC gateway or HLS/DASH converter
      
      const video = videoRef.current;
      
      // Try to load the stream
      // Note: This is a simplified approach - in production you'd need:
      // 1. A WebRTC gateway (like Kurento, Janus, etc.)
      // 2. Or an RTSP-to-HLS/DASH converter
      // 3. Or use MSE (Media Source Extensions) with converted streams
      
      video.src = rtspUrl;
      video.load();
      
      video.onloadstart = () => {
        console.log("Starting to load RTSP stream...");
      };
      
      video.oncanplay = () => {
        setIsConnected(true);
        console.log("RTSP stream ready to play");
      };
      
      video.onerror = (e) => {
        const errorType = e instanceof Event ? e.type : "unknown";
        setError(`Failed to load stream: ${errorType}`);
        setIsConnected(false);
      };
      
      // Auto-play the stream
      video.play().catch((err) => {
        setError(`Playback error: ${err.message}`);
      });
      
    } catch (err) {
      setError(`Connection error: ${err instanceof Error ? err.message : String(err)}`);
      setIsConnected(false);
    }
  };

  const disconnectFromRTSP = () => {
    if (videoRef.current) {
      videoRef.current.pause();
      videoRef.current.src = "";
      videoRef.current.load();
    }
    setIsConnected(false);
    setError(null);
  };

  return (
    <div style={{ padding: "1rem", height: "100%", display: "flex", flexDirection: "column" }}>
      <h2>RTSP Stream Viewer</h2>
      
      {/* RTSP URL Input */}
      <div style={{ marginBottom: "1rem" }}>
        <label htmlFor="rtsp-url" style={{ display: "block", marginBottom: "0.5rem" }}>
          RTSP URL:
        </label>
        <div style={{ display: "flex", gap: "0.5rem" }}>
          <input
            id="rtsp-url"
            type="text"
            value={rtspUrl}
            onChange={(e) => setRtspUrl(e.target.value)}
            placeholder="rtsp://example.com/stream"
            style={{
              flex: 1,
              padding: "0.5rem",
              border: "1px solid #ccc",
              borderRadius: "4px",
            }}
            disabled={isConnected}
          />
          <button
            onClick={isConnected ? disconnectFromRTSP : connectToRTSP}
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
          <div>URL: {rtspUrl}</div>
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
