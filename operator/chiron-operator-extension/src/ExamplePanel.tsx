import { Immutable, MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import L, { LatLngExpression, Map as LeafletMap, Marker as LeafletMarker } from "leaflet";

// Minimal Leaflet CSS injection to avoid bundling external CSS file
const injectLeafletStyles = () => {
  const id = "leaflet-css-inline";
  if (document.getElementById(id)) return;
  const style = document.createElement("style");
  style.id = id;
  style.textContent = `
  .leaflet-pane,.leaflet-tile,.leaflet-marker-icon,.leaflet-marker-shadow,.leaflet-tile-container,
  .leaflet-pane > svg,.leaflet-pane > canvas,.leaflet-zoom-box,
  .leaflet-image-layer,.leaflet-layer,.leaflet-overlay-pane svg,
  .leaflet-overlay-pane canvas { position:absolute; left:0; top:0; }
  .leaflet-container { overflow:hidden; outline:0; }
  .leaflet-tile { filter: inherit; visibility: hidden; }
  .leaflet-tile-loaded { visibility: inherit; }
  .leaflet-zoom-animated { transform-origin: 0 0; }
  .leaflet-zoom-anim .leaflet-zoom-animated { transition: transform 250ms cubic-bezier(0,0,0.25,1); }
  .leaflet-zoom-anim .leaflet-tile { transition: transform 250ms cubic-bezier(0,0,0.25,1); }
  .leaflet-fade-anim .leaflet-tile { will-change: opacity; }
  .leaflet-fade-anim .leaflet-tile-loaded { transition: opacity 0.2s linear; opacity: 1; }
  .leaflet-control { pointer-events: auto; }
  .leaflet-marker-icon { display: block; }
  .leaflet-container a { color: #0078a8; }
  .leaflet-bar { box-shadow: 0 1px 5px rgba(0,0,0,0.65); border-radius:4px; }
  .leaflet-bar a, .leaflet-bar a:hover { background-color:#fff; border-bottom:1px solid #ccc; width:26px; height:26px; line-height:26px; display:block; text-align:center; text-decoration:none; }
  .leaflet-bar a:last-child { border-bottom:none; }
  .leaflet-control-zoom-in, .leaflet-control-zoom-out { font: bold 18px 'Lucida Console', Monaco, monospace; }
  `;
  document.head.appendChild(style);
};

type GpsFix = { lat: number; lon: number } | undefined;
type WaypointMode = boolean; // true if in waypoint mode

function ExamplePanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [messages, setMessages] = useState<undefined | Immutable<MessageEvent[]>>();

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Map refs
  const mapRef = useRef<LeafletMap | null>(null);
  const droneMarkerRef = useRef<LeafletMarker | null>(null);
  const [hasInternet, setHasInternet] = useState<boolean>(true);
  const [gps, setGps] = useState<GpsFix>(undefined);
  const [waypointMode, setWaypointMode] = useState<WaypointMode>(false);
  const [targetLat, setTargetLat] = useState<string>("37.7749");
  const [targetLon, setTargetLon] = useState<string>("-122.4194");
  const [status, setStatus] = useState<string>("");

  // topic names (assumption, adjust to your system if different)
  const GPS_TOPIC = "/gps/fix"; // sensor_msgs/NavSatFix
  const MODE_TOPIC = "/vehicle/mode"; // std_msgs/String or custom
  const WAYPOINT_TOPIC = "/mission/next_waypoint"; // geometry_msgs/Point or PoseStamped-like

  // subscribe shape memo
  const subscriptions = useMemo(() => [{ topic: GPS_TOPIC }, { topic: MODE_TOPIC }], []);

  // We use a layout effect to setup render handling for our panel. We also setup some topic subscriptions.
  useLayoutEffect(() => {
    injectLeafletStyles();
    // The render handler is run by the broader Foxglove system during playback when your panel
    // needs to render because the fields it is watching have changed. How you handle rendering depends on your framework.
    // You can only setup one render handler - usually early on in setting up your panel.
    //
    // Without a render handler your panel will never receive updates.
    //
    // The render handler could be invoked as often as 60hz during playback if fields are changing often.
    context.onRender = (renderState, done) => {
      // render functions receive a _done_ callback. You MUST call this callback to indicate your panel has finished rendering.
      // Your panel will not receive another render callback until _done_ is called from a prior render. If your panel is not done
      // rendering before the next render call, Foxglove shows a notification to the user that your panel is delayed.
      //
      // Set the done callback into a state variable to trigger a re-render.
      setRenderDone(() => done);

      setMessages(renderState.currentFrame);
    };

    // After adding a render handler, you must indicate which fields from RenderState will trigger updates.
    // If you do not watch any fields then your panel will never render since the panel context will assume you do not want any updates.

    // tell the panel context we want messages for the current frame for topics we've subscribed to
    // This corresponds to the _currentFrame_ field of render state.
    context.watch("currentFrame");

    // subscribe to some topics, you could do this within other effects, based on input fields, etc
    // Once you subscribe to topics, currentFrame will contain message events from those topics (assuming there are messages).
    context.subscribe(subscriptions);

    // Advertise waypoint topic for publishing if supported by runtime
    try {
      // geometry_msgs/Point schema name for compatibility with ROS-like schemas
      (context as any).advertise?.(WAYPOINT_TOPIC, "geometry_msgs/Point");
    } catch {
      // ignore if advertise not available
    }
  }, [context]);

  // invoke the done callback once the render is complete
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Process new messages for GPS and mode
  useEffect(() => {
    if (!messages) return;
    for (const evt of messages) {
      if (evt.topic === GPS_TOPIC) {
        // Expect sensor_msgs/NavSatFix-like structure from Foxglove
        const m = evt.message as any;
        const lat = m.latitude ?? m.lat ?? m.lat_deg;
        const lon = m.longitude ?? m.lon ?? m.lon_deg;
        if (typeof lat === "number" && typeof lon === "number") {
          setGps({ lat, lon });
        }
      } else if (evt.topic === MODE_TOPIC) {
        const m = evt.message as any;
        const text = m.data ?? m.mode ?? m.state ?? "";
        const inWp = typeof text === "string" ? /waypoint|mission|auto/i.test(text) : !!text;
        setWaypointMode(inWp);
      }
    }
  }, [messages]);

  // Initialize map
  useEffect(() => {
    if (mapRef.current) return;
    const el = document.getElementById("map-root");
    if (!el) return;

  const start: LatLngExpression = [Number(targetLat) || 37.7749, Number(targetLon) || -122.4194];
  // Aim for ~30m box: use a high zoom level; refine based on latitude if needed.
  const initialZoom = 20; // ~ 1.1m per pixel at equator at z=20; typical panels will show <30m box.
  const map = L.map(el, { center: start, zoom: initialZoom, zoomControl: true, maxZoom: 22 });
    mapRef.current = map;

    // Esri World Imagery satellite tiles (public). If offline, tile loads fail; we detect via error.
    // Attribution kept for licensing compliance.
  const url = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";
  const attribution = "Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community";
  const tile = L.tileLayer(url, { maxZoom: 22, attribution });
    tile.addTo(map);

    tile.on("tileerror", () => {
      setHasInternet(false);
      setStatus("Offline: using last cached snapshot if available.");
    });
    tile.on("load", () => {
      setHasInternet(true);
      setStatus("");
    });

    // Click handler to set waypoint when in waypoint mode
    map.on("click", (e: any) => {
      const { lat, lng } = e.latlng;
      // Publish waypoint: geometry_msgs/Point-like
      try {
        (context as any).publish?.(WAYPOINT_TOPIC, { x: lng, y: lat, z: 0 });
      } catch {
        // fallback no-op
      }
      setStatus(
        `${waypointMode ? "[WAYPOINT]" : "[DEBUG]"} Click published to ${WAYPOINT_TOPIC}: ${lat.toFixed(
          6,
        )}, ${lng.toFixed(6)}`,
      );
    });

    return () => {
      map.remove();
      mapRef.current = null;
    };
  }, [context, targetLat, targetLon, waypointMode]);

  // Update drone marker and pan
  useEffect(() => {
    const map = mapRef.current;
    if (!map || !gps) return;
    const ll: LatLngExpression = [gps.lat, gps.lon];
    if (!droneMarkerRef.current) {
      droneMarkerRef.current = L.marker(ll, { title: "UAV" }).addTo(map);
    } else {
      droneMarkerRef.current.setLatLng(ll);
    }
    // keep in view
    map.panTo(ll, { animate: true });
  }, [gps]);

  // Fetch and cache snapshot of current target
  const takeSnapshot = async () => {
    try {
      const map = mapRef.current;
      if (!map) return;
      const center = map.getCenter();
      const key = "offlineSnapshot";
      // We cannot do cross-origin canvas reliably with public tiles; instead store center coords only.
      // As a lightweight fallback, we keep a data URL if available via leaflet's built-in tile cache is not accessible.
      // For now we store last position; UI will show a note and keep the map container for cached tiles in browser cache.
      localStorage.setItem(key, JSON.stringify({ lat: center.lat, lon: center.lng, ts: Date.now() }));
      setStatus("Snapshot position cached for offline use.");
    } catch (err) {
      setStatus("Failed to cache snapshot position.");
    }
  };

  // Load cached center if offline
  useEffect(() => {
    if (hasInternet) return;
    const raw = localStorage.getItem("offlineSnapshot");
    if (!raw) return;
    try {
      const { lat, lon } = JSON.parse(raw);
      const map = mapRef.current;
      if (map && typeof lat === "number" && typeof lon === "number") {
        map.setView([lat, lon], 15);
      }
    } catch {}
  }, [hasInternet]);

  const goToTarget = () => {
    const lat = Number(targetLat);
    const lon = Number(targetLon);
    if (Number.isFinite(lat) && Number.isFinite(lon) && mapRef.current) {
      mapRef.current.setView([lat, lon], 20);
      setStatus("Panned to target.");
    } else {
      setStatus("Invalid target lat/lon.");
    }
  };

  return (
    <div style={{ padding: "0.5rem", height: "100%", display: "flex", flexDirection: "column", gap: 8 }}>
      <div style={{ display: "flex", gap: 8, alignItems: "center" }}>
        <strong>Map</strong>
        <span style={{ fontSize: 12, color: waypointMode ? "#14866d" : "#a94442" }}>
          Mode: {waypointMode ? "Waypoint-enabled" : "Manual/Other"}
        </span>
        <span style={{ marginLeft: 8, fontSize: 12, color: hasInternet ? "#666" : "#a94442" }}>
          {hasInternet ? "Online" : "Offline"}
        </span>
        <div style={{ marginLeft: "auto", display: "flex", gap: 8 }}>
          <input style={{ width: 120 }} placeholder="lat" value={targetLat} onChange={(e) => setTargetLat(e.target.value)} />
          <input style={{ width: 120 }} placeholder="lon" value={targetLon} onChange={(e) => setTargetLon(e.target.value)} />
          <button onClick={goToTarget}>Go</button>
          <button onClick={takeSnapshot}>Cache snapshot</button>
        </div>
      </div>
      {status && <div style={{ fontSize: 12, color: "#666" }}>{status}</div>}
      <div id="map-root" style={{ flex: 1, minHeight: 300, border: "1px solid #ddd", borderRadius: 4 }} />
      <div style={{ display: "flex", gap: 12, fontSize: 12 }}>
        <div>
          UAV: {gps ? `${gps.lat.toFixed(6)}, ${gps.lon.toFixed(6)}` : "—"}
        </div>
        <div>Tiles: Esri World Imagery</div>
      </div>
    </div>
  );
}

export function initExamplePanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<ExamplePanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
