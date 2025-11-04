import {
  Immutable,
  MessageEvent,
  PanelExtensionContext,
} from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import L, { LatLngExpression, Map as LeafletMap, Polygon, Marker as LeafletMarker } from "leaflet";

// Minimal Leaflet CSS injection
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

type GeofencePoint = { lat: number; lon: number };
type HumanGPS = { lat: number; lon: number; id: string; timestamp: number };

function GeofenceHumanPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [messages, setMessages] = useState<undefined | Immutable<MessageEvent[]>>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Map refs
  const mapRef = useRef<LeafletMap | null>(null);
  const geofencePolygonRef = useRef<Polygon | null>(null);
  const geofenceMarkersRef = useRef<Map<number, LeafletMarker>>(new Map());
  const humanMarkersRef = useRef<Map<string, LeafletMarker>>(new Map());
  const [status, setStatus] = useState<string>("Initializing...");

  // Topic names
  const GEOFENCE_TOPIC = "/robot_1/mavros/geofence/fences";
  const TARGET_GPS_TOPIC = "/target_gps";
  const TARGET_GPS_LIST_TOPIC = "/target_gps_list";
  const PRECISE_TARGET_GPS_TOPIC = "/precise_target_gps";

  const subscriptions = useMemo(
    () => [
      { topic: GEOFENCE_TOPIC },
      { topic: TARGET_GPS_TOPIC },
      { topic: TARGET_GPS_LIST_TOPIC },
      { topic: PRECISE_TARGET_GPS_TOPIC },
    ],
    []
  );

  // Setup render handler and subscriptions
  useLayoutEffect(() => {
    injectLeafletStyles();

    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      setMessages(renderState.currentFrame);
    };

    context.watch("currentFrame");
    context.subscribe(subscriptions);

    return () => {
      // Cleanup
      if (mapRef.current) {
        mapRef.current.remove();
        mapRef.current = null;
      }
    };
  }, [context, subscriptions]);

  // Initialize map
  useEffect(() => {
    if (mapRef.current) return;

    const el = document.getElementById("map-container");
    if (!el) return;

    // Create map centered on default location
    const map = L.map(el, {
      center: [40.414, -79.947],
      zoom: 15,
      zoomControl: true,
    });

    // Add OpenStreetMap tile layer
    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>',
      maxZoom: 19,
    }).addTo(map);

    mapRef.current = map;
    setStatus("Map loaded");

    // Handle resize
    const resizeObserver = new ResizeObserver(() => {
      map.invalidateSize();
    });
    resizeObserver.observe(el);

    return () => {
      resizeObserver.disconnect();
      map.remove();
      mapRef.current = null;
    };
  }, []);

  // Process messages and update visualization
  useEffect(() => {
    if (!messages || !mapRef.current) {
      renderDone?.();
      return;
    }

    const map = mapRef.current;

    // Process each message
    for (const messageEvent of messages) {
      const topic = messageEvent.topic;
      const message = messageEvent.message;

      try {
        // Process geofence topic
        if (topic === GEOFENCE_TOPIC) {
          const waypointList = message as any;
          if (waypointList.waypoints && Array.isArray(waypointList.waypoints)) {
            const points: GeofencePoint[] = [];
            for (const wp of waypointList.waypoints) {
              if (wp.x_lat !== undefined && wp.y_long !== undefined) {
                points.push({ lat: wp.x_lat, lon: wp.y_long });
              }
            }

            if (points.length >= 3) {
              // Remove old polygon and markers
              if (geofencePolygonRef.current) {
                map.removeLayer(geofencePolygonRef.current);
              }
              geofenceMarkersRef.current.forEach((marker: LeafletMarker) => {
                map.removeLayer(marker);
              });
              geofenceMarkersRef.current.clear();

              // Create new polygon
              const latlngs: LatLngExpression[] = points.map((p) => [p.lat, p.lon]);
              // Close the polygon
              if (latlngs.length > 0 && latlngs[0] !== undefined) {
                latlngs.push(latlngs[0]);
              }

              const polygon = L.polygon(latlngs, {
                color: "#ff0000",
                fillColor: "#ff0000",
                fillOpacity: 0.2,
                weight: 3,
              }).addTo(map);

              geofencePolygonRef.current = polygon;

              // Add markers for each geofence point
              points.forEach((point, index) => {
                const marker = L.marker([point.lat, point.lon], {
                  icon: L.divIcon({
                    className: "geofence-marker",
                    html: `<div style="background-color: #ff0000; width: 12px; height: 12px; border-radius: 50%; border: 2px solid white; box-shadow: 0 0 0 2px #ff0000;"></div>`,
                    iconSize: [12, 12],
                    iconAnchor: [6, 6],
                  }),
                })
                  .addTo(map)
                  .bindPopup(`<b>Geofence Point ${index + 1}</b><br>Lat: ${point.lat.toFixed(6)}<br>Lon: ${point.lon.toFixed(6)}`);
                geofenceMarkersRef.current.set(index, marker);
              });

              map.fitBounds(polygon.getBounds(), { padding: [50, 50] });

              setStatus(`Geofence: ${points.length} points`);
            } else if (points.length > 0) {
              // If less than 3 points, just show markers
              geofenceMarkersRef.current.forEach((marker: LeafletMarker) => {
                map.removeLayer(marker);
              });
              geofenceMarkersRef.current.clear();

              points.forEach((point, index) => {
                const marker = L.marker([point.lat, point.lon], {
                  icon: L.divIcon({
                    className: "geofence-marker",
                    html: `<div style="background-color: #ff0000; width: 12px; height: 12px; border-radius: 50%; border: 2px solid white; box-shadow: 0 0 0 2px #ff0000;"></div>`,
                    iconSize: [12, 12],
                    iconAnchor: [6, 6],
                  }),
                })
                  .addTo(map)
                  .bindPopup(`<b>Geofence Point ${index + 1}</b><br>Lat: ${point.lat.toFixed(6)}<br>Lon: ${point.lon.toFixed(6)}`);
                geofenceMarkersRef.current.set(index, marker);
              });

              setStatus(`Geofence: ${points.length} points (need 3+ for polygon)`);
            }
          }
        }

        // Process target GPS topics (sensor_msgs/NavSatFix)
        const navSatFixTopics = [
          TARGET_GPS_TOPIC,
          TARGET_GPS_LIST_TOPIC,
          PRECISE_TARGET_GPS_TOPIC,
        ];

        if (navSatFixTopics.includes(topic)) {
          const navSatFix = message as any;
          if (
            navSatFix.latitude !== undefined &&
            navSatFix.longitude !== undefined &&
            navSatFix.latitude !== 0 &&
            navSatFix.longitude !== 0
          ) {
            const humanGPS: HumanGPS = {
              lat: navSatFix.latitude,
              lon: navSatFix.longitude,
              id: `${topic}-${Date.now()}`,
              timestamp: Date.now(),
            };

            // Use frame_id to distinguish clusters for /target_gps_list
            // Other topics use topic name only (single marker)
            const frameId = navSatFix.header?.frame_id || "";
            const markerId = topic === TARGET_GPS_LIST_TOPIC 
              ? `${topic}-${frameId}`  // Multiple markers for different clusters
              : topic;                  // Single marker for other topics

            if (humanMarkersRef.current.has(markerId)) {
              const marker = humanMarkersRef.current.get(markerId)!;
              const currentLatLng = marker.getLatLng();
              // Update only if coordinates changed
              if (Math.abs(currentLatLng.lat - humanGPS.lat) > 1e-6 || 
                  Math.abs(currentLatLng.lng - humanGPS.lon) > 1e-6) {
                marker.setLatLng([humanGPS.lat, humanGPS.lon]);
              }
            } else {
              // Set marker color and icon based on topic
              let color = "#3388ff";
              let iconText = "H";
              if (topic === PRECISE_TARGET_GPS_TOPIC) {
                color = "#00ff00";
                iconText = "P";
              } else if (topic === TARGET_GPS_LIST_TOPIC) {
                color = "#ffff00";
                iconText = "L";
              }

              // Create custom icon
              const icon = L.divIcon({
                className: "custom-marker",
                html: `<div style="background-color: ${color}; width: 20px; height: 20px; border-radius: 50%; border: 2px solid white; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 12px;">${iconText}</div>`,
                iconSize: [20, 20],
                iconAnchor: [10, 10],
              });

              const popupText = frameId 
                ? `<b>${topic}</b><br>Cluster: ${frameId}<br>Lat: ${humanGPS.lat.toFixed(6)}<br>Lon: ${humanGPS.lon.toFixed(6)}`
                : `<b>${topic}</b><br>Lat: ${humanGPS.lat.toFixed(6)}<br>Lon: ${humanGPS.lon.toFixed(6)}`;

              const marker = L.marker([humanGPS.lat, humanGPS.lon], { icon })
                .addTo(map)
                .bindPopup(popupText);

              humanMarkersRef.current.set(markerId, marker);
            }

            // Update status
            if (topic === TARGET_GPS_LIST_TOPIC && frameId) {
              const clusterCount = Array.from(humanMarkersRef.current.keys())
                .filter(id => id.startsWith(TARGET_GPS_LIST_TOPIC)).length;
              setStatus(`Target GPS List: ${clusterCount} clusters active`);
            } else {
              setStatus(`Human GPS: ${humanGPS.lat.toFixed(6)}, ${humanGPS.lon.toFixed(6)}`);
            }
          }
        }
      } catch (error) {
        console.error(`Error processing message from ${topic}:`, error);
        setStatus(`Error: ${error}`);
      }
    }

    renderDone?.();
  }, [messages, renderDone]);

  return (
    <div style={{ width: "100%", height: "100%", display: "flex", flexDirection: "column" }}>
      <div style={{ padding: "8px", backgroundColor: "#f0f0f0", borderBottom: "1px solid #ccc" }}>
        <div style={{ fontSize: "12px", fontWeight: "bold", marginBottom: "4px" }}>
          Geofence & Human GPS Visualizer
        </div>
        <div style={{ fontSize: "11px", color: "#666" }}>{status}</div>
        <div style={{ fontSize: "10px", color: "#999", marginTop: "4px" }}>
          Topics: {GEOFENCE_TOPIC}, {TARGET_GPS_TOPIC}, {TARGET_GPS_LIST_TOPIC}, {PRECISE_TARGET_GPS_TOPIC}
        </div>
      </div>
      <div
        id="map-container"
        style={{ flex: 1, width: "100%", height: "100%", position: "relative" }}
      />
    </div>
  );
}

export function initGeofenceHumanPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<GeofenceHumanPanel context={context} />);
  return () => {
    root.unmount();
  };
}

