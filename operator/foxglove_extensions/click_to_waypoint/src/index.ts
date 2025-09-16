// Foxglove Panel extension entry point
// Minimal API surface to support loading as a custom panel without external deps

// Lightweight types for Foxglove extension API (runtime provides actual impl)
export type PanelExtensionContext = {
  registerPanel: (panelId: string, panel: Panel) => void;
};

export type InitializeArgs = {
  document: Document;
  panelElement: HTMLElement & { context?: any };
  // Common Foxglove panel APIs (guarded access since not all exist in dev builds)
  setTitle?: (title: string) => void;
  subscribe?: (topics: string[] | { topic: string }[]) => void;
  onRender?: (cb: (renderState: any) => void) => void;
  addMessageSchema?: (schema: any) => void;
  advertise?: (channel: any) => void;
  publish?: (topic: string, payload: any) => void;
};

export type Panel = {
  initialize: (args: InitializeArgs) => void;
};

import { Map, TileLayer, marker, LatLng, Polygon } from "leaflet";

function metersToLatLonOffsets(metersNorth: number, metersEast: number, latDeg: number): { dLat: number; dLon: number } {
  const latRad = (latDeg * Math.PI) / 180;
  const dLat = metersNorth / 111320; // meters per degree latitude
  const dLon = metersEast / (40075000 * Math.cos(latRad) / 360);
  return { dLat, dLon };
}

function createPanel(): Panel {
  return {
    initialize: ({ document, panelElement, setTitle, publish, subscribe, onRender, addMessageSchema, advertise }) => {
      setTitle?.("Click-to-Waypoint");

      // Root container
      const container = document.createElement("div");
      container.style.position = "absolute";
      container.style.inset = "0";
      panelElement.appendChild(container);

      // Ensure Leaflet CSS is available
      const leafletCssId = "leaflet-css";
      if (!document.getElementById(leafletCssId)) {
        const link = document.createElement("link");
        link.id = leafletCssId;
        link.rel = "stylesheet";
        link.href = "https://unpkg.com/leaflet@1.9.4/dist/leaflet.css";
        document.head.appendChild(link);
      }

      // Toolbar
      const toolbar = document.createElement("div");
      toolbar.style.position = "absolute";
      toolbar.style.top = "8px";
      toolbar.style.left = "8px";
      toolbar.style.zIndex = "1000";
      toolbar.style.background = "rgba(0,0,0,0.6)";
      toolbar.style.color = "#fff";
      toolbar.style.padding = "6px 8px";
      toolbar.style.borderRadius = "6px";
      toolbar.style.font = "12px sans-serif";
      toolbar.innerHTML = `
        <div style="display:flex; gap:8px; align-items:center;">
          <label>Tile source:
            <select id="tileSrc">
              <option value="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png">OSM</option>
              <option value="https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}">Google Sat (demo)</option>
            </select>
          </label>
          <label>30m box: <input id="boxToggle" type="checkbox" checked /></label>
          <button id="clearBtn">Clear</button>
        </div>`;
      container.appendChild(toolbar);

      // Map element
      const mapEl = document.createElement("div");
      mapEl.id = "map";
      mapEl.style.position = "absolute";
      mapEl.style.inset = "0";
      container.appendChild(mapEl);

      // Initialize Leaflet map
      const map = new Map(mapEl, { center: [37.4275, -122.1697], zoom: 18 });
      let tiles = new TileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
        attribution: "&copy; OpenStreetMap contributors",
        maxZoom: 20,
      }).addTo(map);

      const droneMarker = marker([37.4275, -122.1697]);
      droneMarker.addTo(map);

      let box: Polygon | undefined;
      let waypointMarker: ReturnType<typeof marker> | undefined;

      // UI controls
      const tileSrcSel = toolbar.querySelector("#tileSrc") as HTMLSelectElement;
      tileSrcSel.addEventListener("change", () => {
        map.removeLayer(tiles);
        tiles = new TileLayer(tileSrcSel.value, { maxZoom: 20 }).addTo(map);
      });

      const clearBtn = toolbar.querySelector("#clearBtn") as HTMLButtonElement;
      clearBtn.addEventListener("click", () => {
        if (box) {
          map.removeLayer(box);
          box = undefined;
        }
        if (waypointMarker) {
          map.removeLayer(waypointMarker);
          waypointMarker = undefined;
        }
      });

      const boxToggle = toolbar.querySelector("#boxToggle") as HTMLInputElement;

      // Subscribe to drone GNSS position (NavSatFix). If the runtime supports
      // richer subscription objects, a plain topic string still works in most builds.
      subscribe?.(["/fix"]);

      // Handle clicks to set waypoint and 30x30m box
      map.on("click", (e: any) => {
        const latlng: LatLng = e.latlng;
        if (waypointMarker) {
          map.removeLayer(waypointMarker);
        }
        waypointMarker = marker(latlng).addTo(map);

        if (box) {
          map.removeLayer(box);
          box = undefined;
        }
        if (boxToggle.checked) {
          const half = 15; // 30m box
          const { dLat: dn, dLon: de } = metersToLatLonOffsets(half, half, latlng.lat);
          const { dLat: ds, dLon: dw } = metersToLatLonOffsets(-half, -half, latlng.lat);
          const nw = [latlng.lat + dn, latlng.lng + dw] as [number, number];
          const ne = [latlng.lat + dn, latlng.lng + de] as [number, number];
          const se = [latlng.lat + ds, latlng.lng + de] as [number, number];
          const sw = [latlng.lat + ds, latlng.lng + dw] as [number, number];
          box = new Polygon([nw, ne, se, sw]);
          box.addTo(map);
        }

        // Publish a waypoint command.
        // Option A: simple JSON payload (bridge/consumer translates to ROS2)
        const payload = { type: "goto_gps", lat: latlng.lat, lon: latlng.lng, alt: 0 };
        publish?.("/behavior_governor/click_waypoint", payload);

        // Option A2: also publish a std_msgs/String command to behavior_governor/command
        try {
          const stdString = `goto_waypoint ${latlng.lat} ${latlng.lng} 0`;
          const stdStringSchema = `string data`;
          addMessageSchema?.({ name: "std_msgs/msg/String", encoding: "ros2msg", schema: stdStringSchema });
          advertise?.({ topic: "/behavior_governor/command", encoding: "ros2", schemaName: "std_msgs/msg/String" });
          publish?.("/behavior_governor/command", { data: stdString });
        } catch (_) {
          // ignore if advertise/publish is unsupported
        }

        // Option B: advertise a ROS2 message schema and publish a GeoPoseStamped (if supported by runtime)
        try {
          const geoPoseStamped = `
            std_msgs/Header header
            geographic_msgs/GeoPose pose
            ===
            MSG: std_msgs/Header
            builtin_interfaces/Time stamp
            string frame_id
            ===
            MSG: builtin_interfaces/Time
            int32 sec
            uint32 nanosec
            ===
            MSG: geographic_msgs/GeoPose
            geographic_msgs/GeoPoint position
            geometry_msgs/Quaternion orientation
            ===
            MSG: geographic_msgs/GeoPoint
            float64 latitude
            float64 longitude
            float64 altitude
            ===
            MSG: geometry_msgs/Quaternion
            float64 x
            float64 y
            float64 z
            float64 w
          `;
          addMessageSchema?.({ name: "geographic_msgs/msg/GeoPoseStamped", encoding: "ros2msg", schema: geoPoseStamped });
          advertise?.({ topic: "/operator/click_waypoint", encoding: "ros2", schemaName: "geographic_msgs/msg/GeoPoseStamped" });
          const now = new Date();
          const sec = Math.floor(now.getTime() / 1000);
          const nanosec = (now.getTime() % 1000) * 1e6;
          publish?.("/operator/click_waypoint", {
            header: { stamp: { sec, nanosec }, frame_id: "map" },
            pose: {
              position: { latitude: latlng.lat, longitude: latlng.lng, altitude: 0 },
              orientation: { x: 0, y: 0, z: 0, w: 1 },
            },
          });
        } catch (_) {
          // No-op if extension runtime doesn't support advertise/publish to ROS2
        }
      });

      // Render hook to receive messages in current frame
      onRender?.(({ currentFrame }: any) => {
        if (!currentFrame) return;
        const msgs: any[] = currentFrame?.["/fix"] ?? [];
        const last = msgs[msgs.length - 1];
        const nav = last?.message ?? last; // some builds wrap under .message
        if (typeof nav?.latitude === "number" && typeof nav?.longitude === "number") {
          droneMarker.setLatLng([nav.latitude, nav.longitude]);
        }
      });

      // Fallback: older runtimes may provide a context message hook
      panelElement.context?.onMessage?.((topic: string, msg: any) => {
        if (topic === "/fix") {
          if (typeof msg?.latitude === "number" && typeof msg?.longitude === "number") {
            droneMarker.setLatLng([msg.latitude, msg.longitude]);
          }
        }
      });

      // Resize handling
      const resize = () => map.invalidateSize();
      const ResizeObserverCtor = (window as any).ResizeObserver;
      if (ResizeObserverCtor) {
        const ro = new ResizeObserverCtor(resize);
        ro.observe(panelElement);
      } else {
        // Fallback
        window.addEventListener("resize", resize);
      }
    },
  };
}

export function activate(extensionContext: PanelExtensionContext) {
  extensionContext.registerPanel("click-to-waypoint", createPanel());
}
