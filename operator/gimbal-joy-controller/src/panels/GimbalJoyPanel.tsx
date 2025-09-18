import {
  Immutable,
  PanelExtensionContext,
  RenderState,
  SettingsTree,
  SettingsTreeAction,
} from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

// Panel config shape (persisted in layout)
export type GimbalJoyConfig = {
  // Index of preferred gamepad by id substring or exact index
  deviceId?: string; // substring match against Gamepad.id
  deviceIndex?: number; // fallback to index

  // Button indices per Gamepad.buttons[]
  bindPanUp: number;
  bindPanDown: number;
  bindPanLeft: number;
  bindPanRight: number;
  bindZoomIn: number;
  bindZoomOut: number;
  bindTrigger: number;
  bindModeSwitch: number;
  bindRecenter: number;

  // Optional axes for analog control (e.g. left stick)
  axisPanX?: number; // negative left, positive right
  axisPanY?: number; // negative up, positive down
  axisDeadzone?: number; // 0..1
  axisScale?: number; // sensitivity multiplier

  // Topic to publish joystick/gimbal commands to
  // We publish a simple custom schema object unless data source supports ROS publish
  // Users can remap in scripts if needed.
  publishTopic?: string; // e.g. "/gimbal/teleop"
};

const DEFAULT_CONFIG: GimbalJoyConfig = {
  bindPanUp: 12, // D-pad up (standard mapping)
  bindPanDown: 13,
  bindPanLeft: 14,
  bindPanRight: 15,
  bindZoomIn: 7, // RT
  bindZoomOut: 6, // LT
  bindTrigger: 5, // RB
  bindModeSwitch: 9, // START/Menu
  bindRecenter: 3, // Y
  axisPanX: 0, // left stick X
  axisPanY: 1, // left stick Y
  axisDeadzone: 0.2,
  axisScale: 1.0,
  publishTopic: "/gimbal/teleop",
};

// Utility
function applyDeadzone(value: number, dz: number): number {
  if (Math.abs(value) < dz) return 0;
  // rescale so post-deadzone is linear 0..1
  const sign = Math.sign(value);
  const mag = (Math.abs(value) - dz) / (1 - dz);
  return sign * mag;
}

function useAnimationFrame(callback: (timestamp: number) => void): void {
  const cbRef = useRef(callback);
  cbRef.current = callback;
  useEffect(() => {
    let raf = 0;
    const loop = (t: number) => {
      cbRef.current(t);
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf);
  }, []);
}

function listGamepads(): Gamepad[] {
  return (navigator.getGamepads?.() ?? []).filter(Boolean) as Gamepad[];
}

function findSelectedGamepad(cfg: GimbalJoyConfig): Gamepad | undefined {
  const pads = listGamepads();
  if (cfg.deviceId && pads.length > 0) {
    const match = pads.find((p) => p && p.id.toLowerCase().includes(cfg.deviceId!.toLowerCase()));
    if (match) return match;
  }
  if (cfg.deviceIndex !== undefined) {
    return pads[cfg.deviceIndex];
  }
  return pads[0];
}

// Simple event accumulator to emit commands; panels can both publish and also display status
type TeleopCommand = {
  panX: number; // -1..1 left/right
  panY: number; // -1..1 up/down
  zoom: -1 | 0 | 1; // -1 out, +1 in
  trigger: boolean;
  modeSwitch: boolean;
  recenter: boolean;
};

function buildSettingsTree(
  cfg: GimbalJoyConfig,
  setCfg: (partial: Partial<GimbalJoyConfig>) => void,
  availableGamepads: Gamepad[],
): SettingsTree {
  const deviceOptions = availableGamepads.map((p, idx) => ({
    label: `${idx}: ${p.id}`,
    value: `${idx}`,
  }));
  return {
    actionHandler: (action: SettingsTreeAction) => {
      if (action.action !== "update") return;
      const path = action.payload.path;
      const key = path[path.length - 1];
      const value = (action as any).payload.value as any;
      switch (key) {
        case "deviceIndex":
          setCfg({ deviceIndex: value === undefined ? undefined : Number(value) });
          break;
        case "deviceId":
          setCfg({ deviceId: value });
          break;
        case "bindPanUp":
        case "bindPanDown":
        case "bindPanLeft":
        case "bindPanRight":
        case "bindZoomIn":
        case "bindZoomOut":
        case "bindTrigger":
        case "bindModeSwitch":
        case "bindRecenter":
        case "axisPanX":
        case "axisPanY":
        case "axisDeadzone":
        case "axisScale":
        case "publishTopic":
          setCfg({ [key]: typeof value === "string" && key !== "publishTopic" ? Number(value) : value } as any);
          break;
        default:
          break;
      }
    },
    nodes: {
      general: {
        label: "Gimbal Joy Controller",
        fields: {
          deviceIndex: {
            label: "Gamepad",
            input: "select",
            value:
              cfg.deviceIndex !== undefined ? String(cfg.deviceIndex) : availableGamepads[0] ? "0" : undefined,
            options: [{ label: "Auto", value: undefined }, ...deviceOptions],
            help: "Choose which detected gamepad to use (or Auto selects the first).",
          },
          deviceId: {
            label: "Device filter (id substring)",
            input: "string",
            value: cfg.deviceId,
            help: "Optional substring to match Gamepad.id (case-insensitive).",
          },
          publishTopic: { label: "Publish topic", input: "string", value: cfg.publishTopic },
        },
        children: {
          buttons: {
            label: "Button Bindings (indices)",
            fields: {
              bindPanUp: { label: "Pan Up", input: "number", value: cfg.bindPanUp, step: 1, min: 0 },
              bindPanDown: { label: "Pan Down", input: "number", value: cfg.bindPanDown, step: 1, min: 0 },
              bindPanLeft: { label: "Pan Left", input: "number", value: cfg.bindPanLeft, step: 1, min: 0 },
              bindPanRight: { label: "Pan Right", input: "number", value: cfg.bindPanRight, step: 1, min: 0 },
              bindZoomIn: { label: "Zoom In", input: "number", value: cfg.bindZoomIn, step: 1, min: 0 },
              bindZoomOut: { label: "Zoom Out", input: "number", value: cfg.bindZoomOut, step: 1, min: 0 },
              bindTrigger: { label: "Trigger", input: "number", value: cfg.bindTrigger, step: 1, min: 0 },
              bindModeSwitch: { label: "Mode Switch", input: "number", value: cfg.bindModeSwitch, step: 1, min: 0 },
              bindRecenter: { label: "Recenter", input: "number", value: cfg.bindRecenter, step: 1, min: 0 },
            },
          },
          axes: {
            label: "Axes",
            fields: {
              axisPanX: { label: "Pan X Axis", input: "number", value: cfg.axisPanX, step: 1, min: 0 },
              axisPanY: { label: "Pan Y Axis", input: "number", value: cfg.axisPanY, step: 1, min: 0 },
              axisDeadzone: {
                label: "Deadzone",
                input: "number",
                value: cfg.axisDeadzone,
                min: 0,
                max: 1,
                step: 0.05,
              },
              axisScale: { label: "Sensitivity", input: "number", value: cfg.axisScale, min: 0.1, max: 5, step: 0.1 },
            },
          },
        },
      },
    },
  };
}

function GimbalJoyPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [cfg, setCfgState] = useState<GimbalJoyConfig>(() => ({ ...DEFAULT_CONFIG, ...(context.initialState as Partial<GimbalJoyConfig> | undefined) }));
  const [pads, setPads] = useState<Gamepad[]>(listGamepads());
  const [lastButtons, setLastButtons] = useState<boolean[]>([]);
  const [status, setStatus] = useState<string>("Connect a controller and press any button");

  // Save config to layout when changed
  const setCfg = (partial: Partial<GimbalJoyConfig>) => {
    setCfgState((prev) => {
      const next = { ...prev, ...partial };
      context.saveState(next);
      // Update settings editor with new values
      context.updatePanelSettingsEditor(buildSettingsTree(next, setCfg, pads));
      return next;
    });
  };

  // Register settings UI
  useEffect(() => {
    context.updatePanelSettingsEditor(buildSettingsTree(cfg, setCfg, pads));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Gamepad connection events
  useEffect(() => {
    const handleConnect = () => setPads(listGamepads());
    const handleDisconnect = () => setPads(listGamepads());
    window.addEventListener("gamepadconnected", handleConnect);
    window.addEventListener("gamepaddisconnected", handleDisconnect);
    return () => {
      window.removeEventListener("gamepadconnected", handleConnect);
      window.removeEventListener("gamepaddisconnected", handleDisconnect);
    };
  }, []);

  // Render lifecycle
  useLayoutEffect(() => {
    context.onRender = (_renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
    };
    // We don't need to watch topics/time for now; this panel is input-driven
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Poll gamepad state every frame and publish teleop commands
  useAnimationFrame(() => {
    const pad = findSelectedGamepad(cfg);
    if (!pad) {
      setStatus("No gamepad detected");
      return;
    }
    setStatus(`Using: ${pad.id}`);

    const buttons = pad.buttons.map((b) => (typeof b === "object" ? b.pressed : Boolean(b)));
    const axes = pad.axes.slice();

    // Digital zoom
    const zoomIn = buttons[cfg.bindZoomIn] ?? false;
    const zoomOut = buttons[cfg.bindZoomOut] ?? false;
    const zoom: -1 | 0 | 1 = zoomIn && !zoomOut ? 1 : zoomOut && !zoomIn ? -1 : 0;

    // Digital pan deltas
    const panRight = buttons[cfg.bindPanRight] ?? false;
    const panLeft = buttons[cfg.bindPanLeft] ?? false;
    const panUp = buttons[cfg.bindPanUp] ?? false;
    const panDown = buttons[cfg.bindPanDown] ?? false;
    let panX = (panRight ? 1 : 0) + (panLeft ? -1 : 0);
    let panY = (panDown ? 1 : 0) + (panUp ? -1 : 0);

    // Analog override
    if (cfg.axisPanX !== undefined && cfg.axisPanY !== undefined) {
      const ax = applyDeadzone(axes[cfg.axisPanX] ?? 0, cfg.axisDeadzone ?? 0);
      const ay = applyDeadzone(axes[cfg.axisPanY] ?? 0, cfg.axisDeadzone ?? 0);
      // Gamepad Y axis is usually up-negative; invert so up is +
      panX = ax * (cfg.axisScale ?? 1);
      panY = -ay * (cfg.axisScale ?? 1);
    }

  // button states for edge detection are read via edge() below

    // Edge detection for momentary actions (mode switch, recenter, trigger)
    const edge = (idx: number) => {
      const prev = lastButtons[idx] ?? false;
      const now = buttons[idx] ?? false;
      return !prev && now;
    };

    const cmd: TeleopCommand = {
      panX,
      panY,
      zoom,
      trigger: edge(cfg.bindTrigger) ? true : false,
      modeSwitch: edge(cfg.bindModeSwitch) ? true : false,
      recenter: edge(cfg.bindRecenter) ? true : false,
    };

    setLastButtons(buttons);

    // Publish via Foxglove publish API if available
    if (context.publish && context.advertise) {
      try {
        // Advertise once. We can store a flag on context.sharedPanelState
        const advertised = (context as any)._gimbalJoyAdvertised as boolean | undefined;
        if (!advertised) {
          const topic = (cfg.publishTopic ?? DEFAULT_CONFIG.publishTopic) as string;
          context.advertise(topic, "gimbal/Teleop");
          (context as any)._gimbalJoyAdvertised = true;
        }
        context.publish((cfg.publishTopic ?? DEFAULT_CONFIG.publishTopic) as string, cmd);
      } catch (err) {
        // ignore publish errors
      }
    }
  });

  // First-run binding helper: let user quickly set buttons
  const [bindingMode, setBindingMode] = useState<null | keyof GimbalJoyConfig>(null);
  const bindingOrder: Array<keyof GimbalJoyConfig> = [
    "bindPanUp",
    "bindPanDown",
    "bindPanLeft",
    "bindPanRight",
    "bindZoomIn",
    "bindZoomOut",
    "bindTrigger",
    "bindModeSwitch",
    "bindRecenter",
  ];

  const [bindingIdx, setBindingIdx] = useState<number>(0);
  const activePad = useMemo(() => findSelectedGamepad(cfg), [cfg, pads]);

  useAnimationFrame(() => {
    if (bindingMode && activePad) {
      const pressedIdx = activePad.buttons.findIndex((b) => (typeof b === "object" ? b.pressed : Boolean(b)));
      if (pressedIdx >= 0) {
        setCfg({ [bindingMode]: pressedIdx } as any);
        setBindingMode(null);
        setBindingIdx((i) => i + 1);
      }
    }
  });

  useEffect(() => {
    if (bindingIdx < bindingOrder.length) {
      const next = bindingOrder[bindingIdx];
      if (next) setBindingMode(next);
    } else if (bindingMode) {
      setBindingMode(null);
    }
  }, [bindingIdx]);

  const onStartWizard = () => {
    setBindingIdx(0);
    const first = bindingOrder[0];
    if (first) setBindingMode(first);
  };

  return (
    <div style={{ padding: "0.75rem", fontFamily: "sans-serif" }}>
      <h3 style={{ margin: 0 }}>Gimbal Joy Controller</h3>
      <div style={{ color: "#888", marginBottom: 8 }}>{status}</div>
      <div style={{ display: "flex", gap: 8, flexWrap: "wrap", alignItems: "center" }}>
        <button onClick={onStartWizard}>Setup bindings</button>
        <span style={{ fontSize: 12, color: "#666" }}>
          Tip: open panel settings to edit button indices or axes manually.
        </span>
      </div>
      {bindingMode && (
        <div style={{ marginTop: 8, padding: 8, border: "1px dashed #999", borderRadius: 6 }}>
          <b>Press a button for: {String(bindingMode)}</b>
        </div>
      )}
      <div style={{ marginTop: 8, fontSize: 12, color: "#666" }}>
        Actions: Pan Up/Down/Left/Right, Zoom In/Out, Trigger, Mode Switch, Recenter.
      </div>
    </div>
  );
}

export function initGimbalJoyPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<GimbalJoyPanel context={context} />);
  return () => root.unmount();
}
