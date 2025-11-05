import {
  Immutable,
  MessageEvent,
  PanelExtensionContext,
} from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useRef, useState, useCallback } from "react";
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
  const selectedWaypointMarkerRef = useRef<LeafletMarker | null>(null);
  const [status, setStatus] = useState<string>("Initializing...");
  const [selectedWaypoint, setSelectedWaypoint] = useState<{lat: number, lon: number} | null>(null);

  // Topic names
  const GEOFENCE_TOPIC = "/robot_1/mavros/geofence/fences";
  const TARGET_GPS_TOPIC = "/target_gps";
  const TARGET_GPS_LIST_TOPIC = "/target_gps_list";
  const PRECISE_TARGET_GPS_TOPIC = "/precise_target_gps";
  const CASUALTY_GEOLOCATED_TOPIC = "/casualty_geolocated";  // Casualty position
  const SELECTED_WAYPOINT_TOPIC = "/selected_waypoint";  // New topic for selected waypoint
  const DRONE_GPS_TOPIC = "/dtc_mrsd_/mavros/global_position/global";  // Drone position
  const DRONE_HEADING_TOPIC = "/dtc_mrsd_/mavros/global_position/compass_hdg";  // Drone heading

  // Store drone position and heading
  const dronePositionRef = useRef<{lat: number, lon: number} | null>(null);
  const droneHeadingRef = useRef<number | null>(null);  // Heading in degrees (0-360, 0=North)
  const droneMarkerRef = useRef<LeafletMarker | null>(null);
  const mapCenteredOnDroneRef = useRef(false);  // Track if map has been centered on drone
  
  // Throttle updates to prevent high-frequency refreshing
  const lastUpdateTimeRef = useRef<number>(0);
  const UPDATE_THROTTLE_MS = 100; // Update at most every 100ms (10 Hz)

  const subscriptions = useMemo(
    () => [
      { topic: GEOFENCE_TOPIC },
      { topic: TARGET_GPS_TOPIC },
      { topic: TARGET_GPS_LIST_TOPIC },
      { topic: PRECISE_TARGET_GPS_TOPIC },
      { topic: CASUALTY_GEOLOCATED_TOPIC },
      { topic: DRONE_GPS_TOPIC },
      { topic: DRONE_HEADING_TOPIC },
    ],
    []
  );

  // Advertise selected waypoint topic
  const advertisedRef = useRef(false);
  
  const ensureAdvertised = () => {
    if (!context.advertise) return;
    if (advertisedRef.current) return;
    
    try {
      context.advertise(SELECTED_WAYPOINT_TOPIC, "sensor_msgs/NavSatFix");
      advertisedRef.current = true;
      console.log("[GeofenceMap] ✓ Advertised:", SELECTED_WAYPOINT_TOPIC);
    } catch (error) {
      console.error("[GeofenceMap] ✗ Advertise failed:", error);
    }
  };
  
  useEffect(() => {
    ensureAdvertised();
  }, [context]);

  // Publish selected waypoint
  const publishSelectedWaypoint = useCallback((lat: number, lon: number, alt: number = 5.0) => {
    if (!context.publish) return;
    
    ensureAdvertised();

    const msg = {
      header: {
        stamp: { sec: 0, nsec: 0 },
        frame_id: "selected_waypoint",
      },
      status: {
        status: 0,
        service: 1,
      },
      latitude: lat,
      longitude: lon,
      altitude: alt,
      position_covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0],
      position_covariance_type: 0,
    };

    try {
      context.publish(SELECTED_WAYPOINT_TOPIC, msg);
      console.log("[GeofenceMap] ✓ Published:", lat, lon, alt);
      setStatus(`Waypoint: ${lat.toFixed(6)}, ${lon.toFixed(6)}`);
    } catch (error) {
      console.error("[GeofenceMap] ✗ Publish failed:", error);
    }
  }, [context, SELECTED_WAYPOINT_TOPIC]);

  // Zoom to drone position
  const zoomToDrone = () => {
    if (!mapRef.current || !dronePositionRef.current) {
      setStatus("Drone position not available");
      return;
    }
    mapRef.current.setView([dronePositionRef.current.lat, dronePositionRef.current.lon], 18);
    setStatus(`Zoomed to drone: ${dronePositionRef.current.lat.toFixed(6)}, ${dronePositionRef.current.lon.toFixed(6)}`);
  };

  // Zoom to selected waypoint
  const zoomToWaypoint = () => {
    if (!mapRef.current || !selectedWaypoint) {
      setStatus("No waypoint selected");
      return;
    }
    mapRef.current.setView([selectedWaypoint.lat, selectedWaypoint.lon], 18);
    setStatus(`Zoomed to waypoint: ${selectedWaypoint.lat.toFixed(6)}, ${selectedWaypoint.lon.toFixed(6)}`);
  };

  // Setup render handler and subscriptions
  useLayoutEffect(() => {
    injectLeafletStyles();

    context.onRender = (renderState, done) => {
      ensureAdvertised();
      setRenderDone(() => done);
      setMessages(renderState.currentFrame);
    };

    context.watch("currentFrame");
    context.subscribe(subscriptions);

    return () => {
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

    // Create map centered on default location with high zoom for precise small area flight
    const map = L.map(el, {
      center: [40.414, -79.947],
      zoom: 19,  // High zoom level for precise small area operations
      zoomControl: true,
    });

    // Add Satellite tile layer (Esri World Imagery)
    L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", {
      attribution: 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community',
      maxZoom: 19,
    }).addTo(map);

    // Add click handler to set waypoint
    map.on("click", (e) => {
      const { lat, lng } = e.latlng;
      
      // Remove old selected waypoint marker
      if (selectedWaypointMarkerRef.current) {
        map.removeLayer(selectedWaypointMarkerRef.current);
      }

      // Create new selected waypoint marker (small blue dot with "W" - ~0.5m diameter at zoom 18)
      const icon = L.divIcon({
        className: "waypoint-marker",
        html: `<div style="background-color: #0000ff; width: 14px; height: 14px; border-radius: 50%; border: 2px solid white; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 9px; box-shadow: 0 1px 4px rgba(0,0,0,0.5);">W</div>`,
        iconSize: [14, 14],
        iconAnchor: [7, 7],
      });

      const marker = L.marker([lat, lng], { icon })
        .addTo(map)
        .bindPopup(
          `<b>Selected Waypoint</b><br>Lat: ${lat.toFixed(6)}<br>Lon: ${lng.toFixed(6)}<br><br><i>Click "Navigate to Waypoint" in Behavior Tree Controller to go here</i>`
        )
        .openPopup();

      selectedWaypointMarkerRef.current = marker;
      setSelectedWaypoint({ lat, lon: lng });

      // Publish waypoint to ROS topic
      publishSelectedWaypoint(lat, lng, 5.0); // Default altitude 5m
    });

    mapRef.current = map;
    setStatus("Map loaded - Click to set waypoint");

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
  }, [publishSelectedWaypoint]);

  // Process messages and update visualization
  useEffect(() => {
    if (!messages || !mapRef.current) {
      renderDone?.();
      return;
    }

    // Throttle updates to prevent high-frequency DOM manipulation
    const now = Date.now();
    if (now - lastUpdateTimeRef.current < UPDATE_THROTTLE_MS) {
      renderDone?.();
      return;
    }
    lastUpdateTimeRef.current = now;

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

        // Process drone GPS position
        if (topic === DRONE_GPS_TOPIC) {
          const navSatFix = message as any;
          if (
            navSatFix.latitude !== undefined &&
            navSatFix.longitude !== undefined &&
            navSatFix.latitude !== 0 &&
            navSatFix.longitude !== 0
          ) {
            const newLat = navSatFix.latitude;
            const newLon = navSatFix.longitude;

            // Center map on drone position on first GPS fix
            if (!mapCenteredOnDroneRef.current && mapRef.current) {
              mapRef.current.setView([newLat, newLon], 19);
              mapCenteredOnDroneRef.current = true;
              console.log("[GeofenceMap] Map centered on drone:", newLat, newLon);
              setStatus(`Map centered on drone: ${newLat.toFixed(6)}, ${newLon.toFixed(6)}`);
            }

            // Only update if position changed significantly (> 1cm)
            const shouldUpdate = !dronePositionRef.current ||
              Math.abs(dronePositionRef.current.lat - newLat) > 0.0000001 ||
              Math.abs(dronePositionRef.current.lon - newLon) > 0.0000001;

            if (shouldUpdate) {
              dronePositionRef.current = {
                lat: newLat,
                lon: newLon,
              };

              // Update or create drone marker
              const heading = droneHeadingRef.current ?? 0;
              const rotation = heading; // Heading is already in degrees, 0=North
              
              if (droneMarkerRef.current) {
                droneMarkerRef.current.setLatLng([newLat, newLon]);
                // Update icon with current heading
                const droneIcon = L.divIcon({
                  className: "drone-marker",
                  html: `<div style="background-color: #ff6600; width: 24px; height: 24px; border-radius: 50%; border: 2px solid white; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 14px; box-shadow: 0 2px 6px rgba(0,0,0,0.5); transform: rotate(${rotation}deg);">▲</div>`,
                  iconSize: [24, 24],
                  iconAnchor: [12, 12],
                });
                droneMarkerRef.current.setIcon(droneIcon);
                // Update popup content
                droneMarkerRef.current.setPopupContent(
                  `<b>Drone Position</b><br>Lat: ${newLat.toFixed(6)}<br>Lon: ${newLon.toFixed(6)}<br>Heading: ${heading.toFixed(1)}°`
                );
              } else {
                const droneIcon = L.divIcon({
                  className: "drone-marker",
                  html: `<div style="background-color: #ff6600; width: 24px; height: 24px; border-radius: 50%; border: 2px solid white; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 14px; box-shadow: 0 2px 6px rgba(0,0,0,0.5); transform: rotate(${rotation}deg);">▲</div>`,
                  iconSize: [24, 24],
                  iconAnchor: [12, 12],
                });

                const marker = L.marker([newLat, newLon], { icon: droneIcon })
                  .addTo(map)
                  .bindPopup(
                    `<b>Drone Position</b><br>Lat: ${newLat.toFixed(6)}<br>Lon: ${newLon.toFixed(6)}<br>Heading: ${heading.toFixed(1)}°`
                  );

                droneMarkerRef.current = marker;
              }
            }
          }
        }

        // Process drone heading
        if (topic === DRONE_HEADING_TOPIC) {
          const headingMsg = message as any;
          // The message might be std_msgs/Float64 with 'data' field, or might be a direct number
          let heading: number | null = null;
          
          if (typeof headingMsg === 'number') {
            heading = headingMsg;
          } else if (headingMsg.data !== undefined) {
            heading = headingMsg.data;
          }
          
          if (heading !== null && !isNaN(heading)) {
            // Normalize heading to 0-360 range
            heading = ((heading % 360) + 360) % 360;
            
            const headingChanged = droneHeadingRef.current === null || 
              Math.abs(droneHeadingRef.current - heading) > 0.5; // Update if changed by >0.5 degrees
            
            if (headingChanged) {
              droneHeadingRef.current = heading;
              
              // Update drone marker if it exists and has position
              if (droneMarkerRef.current && dronePositionRef.current) {
                const rotation = heading;
                const droneIcon = L.divIcon({
                  className: "drone-marker",
                  html: `<div style="background-color: #ff6600; width: 24px; height: 24px; border-radius: 50%; border: 2px solid white; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 14px; box-shadow: 0 2px 6px rgba(0,0,0,0.5); transform: rotate(${rotation}deg);">▲</div>`,
                  iconSize: [24, 24],
                  iconAnchor: [12, 12],
                });
                droneMarkerRef.current.setIcon(droneIcon);
                droneMarkerRef.current.setPopupContent(
                  `<b>Drone Position</b><br>Lat: ${dronePositionRef.current.lat.toFixed(6)}<br>Lon: ${dronePositionRef.current.lon.toFixed(6)}<br>Heading: ${heading.toFixed(1)}°`
                );
              }
            }
          }
        }

        // Process target GPS topics (sensor_msgs/NavSatFix)
        const navSatFixTopics = [
          TARGET_GPS_TOPIC,
          TARGET_GPS_LIST_TOPIC,
          PRECISE_TARGET_GPS_TOPIC,
          CASUALTY_GEOLOCATED_TOPIC,
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
              } else if (topic === CASUALTY_GEOLOCATED_TOPIC) {
                color = "#ff0000";
                iconText = "C";
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
            } else if (topic === CASUALTY_GEOLOCATED_TOPIC) {
              setStatus(`Casualty Geolocated: ${humanGPS.lat.toFixed(6)}, ${humanGPS.lon.toFixed(6)}`);
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
        <div style={{ display: "flex", alignItems: "center", justifyContent: "space-between", marginBottom: "4px" }}>
          <div style={{ fontSize: "12px", fontWeight: "bold" }}>
            Geofence & Human GPS Visualizer + Waypoint Selector
          </div>
          <div style={{ display: "flex", gap: "4px" }}>
            <button
              onClick={zoomToDrone}
              style={{
                padding: "4px 8px",
                fontSize: "11px",
                borderRadius: 4,
                border: "1px solid #ff6600",
                background: "#fff",
                color: "#ff6600",
                cursor: "pointer",
                fontWeight: "600",
              }}
              title="Zoom to drone position"
            >
              ✈ Drone
            </button>
            <button
              onClick={zoomToWaypoint}
              disabled={!selectedWaypoint}
              style={{
                padding: "4px 8px",
                fontSize: "11px",
                borderRadius: 4,
                border: "1px solid #0000ff",
                background: selectedWaypoint ? "#fff" : "#eee",
                color: selectedWaypoint ? "#0000ff" : "#999",
                cursor: selectedWaypoint ? "pointer" : "not-allowed",
                fontWeight: "600",
              }}
              title="Zoom to selected waypoint"
            >
              📍 Waypoint
            </button>
          </div>
        </div>
        <div style={{ fontSize: "11px", color: "#666" }}>{status}</div>
        {selectedWaypoint && (
          <div style={{ fontSize: "11px", color: "#0000ff", marginTop: "4px", fontWeight: "600" }}>
            📍 Selected Waypoint: {selectedWaypoint.lat.toFixed(6)}, {selectedWaypoint.lon.toFixed(6)} 
            <span style={{ marginLeft: "8px", fontSize: "10px", fontWeight: "normal", color: "#666" }}>
              → Now click "Navigate to Waypoint" in Behavior Tree Controller
            </span>
          </div>
        )}
        <div style={{ fontSize: "10px", color: "#999", marginTop: "4px" }}>
          Click map to select waypoint • Topics: {GEOFENCE_TOPIC}, {SELECTED_WAYPOINT_TOPIC}
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

