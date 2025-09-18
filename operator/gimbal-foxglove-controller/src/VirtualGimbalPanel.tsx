import {
  Immutable,
  PanelExtensionContext,
  RenderState,
  SettingsTree,
  SettingsTreeAction,
} from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { ros1, ros2humble as ros2 } from "@foxglove/rosmsg-msgs-common";
import { createRoot } from "react-dom/client";

type TeleopCommand = {
  panX: number; // -1..1 left/right
  panY: number; // -1..1 up/down (positive is up)
  zoom: -1 | 0 | 1; // -1 out, +1 in
  trigger: boolean;
  modeSwitch: boolean;
  recenter: boolean;
};

type PanelConfig = {
  publishTopic: string; // e.g. "/gimbal/teleop"
  stepSize: number; // magnitude per tick when holding a direction
  repeatHz: number; // publish rate while button is held
};

const DEFAULT_CONFIG: PanelConfig = {
  publishTopic: "/gimbal/teleop",
  stepSize: 1.0,
  repeatHz: 20,
};

function buildSettingsTree(
  cfg: PanelConfig,
  setCfg: (partial: Partial<PanelConfig>) => void,
): SettingsTree {
  return {
    actionHandler: (action: SettingsTreeAction) => {
      if (action.action !== "update") return;
      const key = action.payload.path[action.payload.path.length - 1] as keyof PanelConfig;
      const raw = (action as any).payload.value as any;
      if (key === "publishTopic") setCfg({ publishTopic: String(raw ?? "") });
      if (key === "stepSize") setCfg({ stepSize: Number(raw ?? 1) });
      if (key === "repeatHz") setCfg({ repeatHz: Number(raw ?? 20) });
    },
    nodes: {
      general: {
        label: "Gimbal Virtual Controller",
        fields: {
          publishTopic: { label: "Publish topic", input: "string", value: cfg.publishTopic },
          stepSize: { label: "Pan step (0-1)", input: "number", value: cfg.stepSize, min: 0, max: 1, step: 0.1 },
          repeatHz: { label: "Repeat rate (Hz)", input: "number", value: cfg.repeatHz, min: 1, max: 60, step: 1 },
        },
      },
    },
  };
}

function useInterval(callback: () => void, hz: number, active: boolean): void {
  const cbRef = useRef(callback);
  cbRef.current = callback;
  useEffect(() => {
    if (!active || hz <= 0) return;
    const intervalMs = 1000 / hz;
    const id = setInterval(() => cbRef.current(), intervalMs);
    return () => clearInterval(id);
  }, [hz, active]);
}

function Button({ label, onPress, onRelease, style }: {
  label: string;
  onPress: () => void;
  onRelease: () => void;
  style?: React.CSSProperties;
}): ReactElement {
  const [down, setDown] = useState(false);
  const start = () => {
    setDown(true);
    onPress();
  };
  const end = () => {
    setDown(false);
    onRelease();
  };
  return (
    <button
      onMouseDown={start}
      onMouseUp={end}
      onMouseLeave={down ? end : undefined}
      onTouchStart={(e) => {
        e.preventDefault();
        start();
      }}
      onTouchEnd={(e) => {
        e.preventDefault();
        end();
      }}
      style={{
        padding: "10px 14px",
        borderRadius: 8,
        border: "1px solid #888",
        background: down ? "#cde" : "#eef",
        cursor: "pointer",
        fontWeight: 600,
        ...style,
      }}
    >
      {label}
    </button>
  );
}

function VirtualGimbalPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [cfg, setCfgState] = useState<PanelConfig>(() => ({
    ...DEFAULT_CONFIG,
    ...(context.initialState as Partial<PanelConfig> | undefined),
  }));

  // pressed state
  const [up, setUp] = useState(false);
  const [down, setDown] = useState(false);
  const [left, setLeft] = useState(false);
  const [right, setRight] = useState(false);
  const [zoomIn, setZoomIn] = useState(false);
  const [zoomOut, setZoomOut] = useState(false);
  const [status] = useState<string>("Hold buttons to send commands");

  // persist config
  const setCfg = (partial: Partial<PanelConfig>) => {
    setCfgState((prev) => {
      const next = { ...prev, ...partial };
      context.saveState(next);
      context.updatePanelSettingsEditor(buildSettingsTree(next, setCfg));
      return next;
    });
  };

  useEffect(() => {
    context.updatePanelSettingsEditor(buildSettingsTree(cfg, setCfg));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Render lifecycle: ensure we (re)advertise when data source changes and call done
  useLayoutEffect(() => {
    context.watch("topics");
    context.onRender = (_rs: Immutable<RenderState>, done) => {
      // attempt advertise if not yet successful
      ensureAdvertised();
      setRenderDone(() => done);
    };
  }, [context]);
  useEffect(() => void renderDone?.(), [renderDone]);

  // Detect profile and advertise appropriate schema
  const advertisedRef = useRef<{ topic?: string; mode?: "ros" | "custom" }>({});
  const useRos = context.dataSourceProfile === "ros1" || context.dataSourceProfile === "ros2";

  const ensureAdvertised = () => {
    const mode: "ros" | "custom" = useRos ? "ros" : "custom";
    const topic = cfg.publishTopic;
    if (!context.advertise) return; // publish not supported (yet)
    if (advertisedRef.current.topic === topic && advertisedRef.current.mode === mode) return;
    try {
      if (mode === "ros") {
        const dt = context.dataSourceProfile === "ros1" ? ros1 : ros2;
        const datatypes = new Map<string, unknown>([
          ["std_msgs/Header", (dt as any)["std_msgs/Header"]],
          ["sensor_msgs/Joy", (dt as any)["sensor_msgs/Joy"]],
        ]);
        try {
          context.advertise(topic, "sensor_msgs/Joy", { datatypes });
        } catch {
          context.advertise(topic, "sensor_msgs/Joy");
        }
      } else {
        context.advertise(topic, "gimbal/Teleop");
      }
      advertisedRef.current = { topic, mode };
    } catch {
      // ignore
    }
  };

  useEffect(() => {
    ensureAdvertised();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [cfg.publishTopic, useRos]);

  const publish = (cmd: TeleopCommand) => {
    try {
      if (useRos) {
        // Publish as sensor_msgs/Joy
        const now = new Date();
        const msg = {
          header: {
            stamp: { sec: Math.floor(now.getTime() / 1000), nsec: (now.getTime() % 1000) * 1e6 },
            frame_id: "",
          },
          axes: [cmd.panX, cmd.panY, cmd.zoom],
          buttons: [cmd.trigger ? 1 : 0, cmd.modeSwitch ? 1 : 0, cmd.recenter ? 1 : 0],
        };
        context.publish?.(cfg.publishTopic, msg);
      } else {
        // Publish custom command
        context.publish?.(cfg.publishTopic, cmd);
      }
    } catch {
      // ignore publish errors
    }
  };

  // Continuous publisher while holding buttons
  const activeHold = up || down || left || right || zoomIn || zoomOut;
  useInterval(() => {
    const panY = (up ? 1 : 0) + (down ? -1 : 0);
    const panX = (right ? 1 : 0) + (left ? -1 : 0);
    const scale = cfg.stepSize;
    const zoom: -1 | 0 | 1 = zoomIn && !zoomOut ? 1 : zoomOut && !zoomIn ? -1 : 0;
    const cmd: TeleopCommand = {
      panX: panX * scale,
      panY: panY * scale,
      zoom,
      trigger: false,
      modeSwitch: false,
      recenter: false,
    };
    publish(cmd);
  }, cfg.repeatHz, activeHold);

  const sendEdge = (partial: Partial<TeleopCommand>) => {
    publish({ panX: 0, panY: 0, zoom: 0, trigger: false, modeSwitch: false, recenter: false, ...partial });
  };

  const grid: React.CSSProperties = {
    display: "grid",
    gridTemplateColumns: "repeat(3, 100px)",
    gridTemplateRows: "repeat(3, 60px)",
    gap: 8,
    alignItems: "center",
    justifyItems: "center",
  };

  return (
    <div style={{ padding: 12, fontFamily: "sans-serif" }}>
      <h3 style={{ margin: 0 }}>Gimbal Virtual Controller</h3>
      <div style={{ color: "#666", marginBottom: 8 }}>
        {status} · Data source: {context.dataSourceProfile ?? "unknown"} · Publish: {context.publish ? "available" : "unavailable"}
      </div>
      <div style={{ display: "flex", gap: 12, flexWrap: "wrap", alignItems: "center" }}>
        <div style={grid}>
          <div />
          <Button label="Up" onPress={() => setUp(true)} onRelease={() => setUp(false)} />
          <div />

          <Button label="Left" onPress={() => setLeft(true)} onRelease={() => setLeft(false)} />
          <div />
          <Button label="Right" onPress={() => setRight(true)} onRelease={() => setRight(false)} />

          <div />
          <Button label="Down" onPress={() => setDown(true)} onRelease={() => setDown(false)} />
          <div />
        </div>

        <div style={{ display: "grid", gridTemplateRows: "repeat(3, 40px)", gap: 8 }}>
          <Button label="Zoom +" onPress={() => setZoomIn(true)} onRelease={() => setZoomIn(false)} />
          <div style={{ textAlign: "center", fontSize: 12, color: "#666" }}>Zoom</div>
          <Button label="Zoom -" onPress={() => setZoomOut(true)} onRelease={() => setZoomOut(false)} />
        </div>

        <div style={{ display: "grid", gap: 8 }}>
          <Button label="Trigger" onPress={() => sendEdge({ trigger: true })} onRelease={() => {}} />
          <Button label="Mode" onPress={() => sendEdge({ modeSwitch: true })} onRelease={() => {}} />
          <Button label="Recenter" onPress={() => sendEdge({ recenter: true })} onRelease={() => {}} />
        </div>
      </div>
      <div style={{ marginTop: 10, fontSize: 12, color: "#666" }}>
        Hold direction buttons to continuously pan or zoom. Click other buttons for one-shot actions.
      </div>
    </div>
  );
}

export function initVirtualGimbalPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<VirtualGimbalPanel context={context} />);
  return () => root.unmount();
}
