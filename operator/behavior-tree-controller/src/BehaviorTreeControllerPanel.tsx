import {
  Immutable,
  PanelExtensionContext,
  RenderState,
  SettingsTree,
  SettingsTreeAction,
} from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

// Supported behavior commands
const BEHAVIOR_COMMANDS = [
  { name: "Arm Commanded", label: "Arm", description: "Arm the drone" },
  { name: "Disarm Commanded", label: "Disarm", description: "Disarm the drone" },
  { name: "Auto Takeoff Commanded", label: "Auto Takeoff", description: "Auto takeoff the drone" },
  { name: "AutoLand Commanded", label: "Auto Land", description: "Auto land the drone" },
  { name: "EStop Commanded", label: "EStop", description: "E-stop the drone" },
  // { name: "Survey Commanded", label: "Survey", description: "Execute survey task" },
  // { name: "Geofence Mapping Commanded", label: "Geofence Mapping", description: "Execute geofence mapping" },
  { name: "Go to Waypoint Commanded", label: "Go to Waypoint", description: "Go to waypoint" },
] as const;

type PanelConfig = {
  publishTopic: string; // e.g. "/dtc_mrsd_/behavior_tree_commands"
  robotNamespace: string; // e.g. "dtc_mrsd_"
};

const DEFAULT_CONFIG: PanelConfig = {
  publishTopic: "/behavior_tree_commands",
  robotNamespace: "",
};

function buildSettingsTree(
  cfg: PanelConfig,
  setCfg: (partial: Partial<PanelConfig>) => void,
): SettingsTree {
  return {
    actionHandler: (action: SettingsTreeAction) => {
      if (action.action !== "update") {
        return;
      }
      const key = action.payload.path[action.payload.path.length - 1] as keyof PanelConfig;
      const raw = action.payload.value as unknown;
      if (key === "publishTopic") {
        const value = typeof raw === "string" ? raw : String(raw ?? "");
        setCfg({ publishTopic: value });
      }
      if (key === "robotNamespace") {
        const value = typeof raw === "string" ? raw : String(raw ?? "");
        const ns = value.trim();
        setCfg({
          robotNamespace: ns,
          publishTopic: ns ? `/${ns}/behavior_tree_commands` : "/behavior_tree_commands",
        });
      }
    },
    nodes: {
      general: {
        label: "Behavior Tree Controller",
        fields: {
          robotNamespace: { 
            label: "Robot Namespace", 
            input: "string", 
            value: cfg.robotNamespace,
            help: "For example: dtc_mrsd_ (without trailing slash)"
          },
          publishTopic: { 
            label: "Publish Topic", 
            input: "string", 
            value: cfg.publishTopic,
            help: "Full topic path, for example: /dtc_mrsd_/behavior_tree_commands"
          },
        },
      },
    },
  };
}

function BehaviorTreeControllerPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [cfg, setCfgState] = useState<PanelConfig>(() => ({
    ...DEFAULT_CONFIG,
    ...(context.initialState as Partial<PanelConfig> | undefined),
  }));
  const [lastSentCommand, setLastSentCommand] = useState<string>("");
  const [status, setStatus] = useState<string>("Ready");

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

  // Render lifecycle
  useLayoutEffect(() => {
    context.watch("topics");
    context.watch("currentFrame"); // 监听连接状态
    context.onRender = (_rs: Immutable<RenderState>, done) => {
      ensureAdvertised();
      setRenderDone(() => done);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [context]);
  useEffect(() => {
    if (renderDone) {
      renderDone();
    }
  }, [renderDone]);

  // Detect profile and advertise appropriate schema
  const advertisedRef = useRef<{ topic?: string; mode?: "ros" | "custom" }>({});
  // 强制使用 ROS 模式（修复 dataSourceProfile = undefined 的问题）
  const useRos = context.dataSourceProfile === "ros1" || context.dataSourceProfile === "ros2" || context.dataSourceProfile === undefined;

  const ensureAdvertised = () => {
    const mode: "ros" | "custom" = useRos ? "ros" : "custom";
    const topic = cfg.publishTopic;
    if (!context.advertise) {
      console.warn("[BehaviorTree] advertise API not available");
      setStatus("发布功能不可用");
      return;
    }
    // 关键：像 gimbal 一样检查 topic 和 mode
    if (advertisedRef.current.topic === topic && advertisedRef.current.mode === mode) {
      console.log("[BehaviorTree] Topic already advertised:", topic, "mode:", mode);
      return;
    }

    console.log("[BehaviorTree] Attempting to advertise topic:", topic);
    console.log("[BehaviorTree] Data source profile:", context.dataSourceProfile);
    console.log("[BehaviorTree] Mode:", mode);

    try {
      if (mode === "ros") {
        console.log("[BehaviorTree] Calling context.advertise with behavior_tree_msgs/BehaviorTreeCommands (ROS mode)");
        // 尝试不提供 datatypes，让 bridge 从环境中解析
        context.advertise(topic, "behavior_tree_msgs/BehaviorTreeCommands");
      } else {
        console.log("[BehaviorTree] Calling context.advertise with behavior_tree/Commands (custom mode)");
        context.advertise(topic, "behavior_tree/Commands");
      }
      advertisedRef.current = { topic, mode };
      console.log("[BehaviorTree] ✓ Successfully advertised:", topic, "mode:", mode);
      setStatus(`已连接到: ${topic}`);
    } catch (err) {
      const errMsg = err instanceof Error ? err.message : String(err);
      console.error("[BehaviorTree] ✗ Failed to advertise:", errMsg);
      setStatus(`广告失败: ${errMsg}`);
    }
  };

  useEffect(() => {
    // 当 topic 或 useRos 改变时，清除广告状态并重新广告
    console.log("[BehaviorTree] Config or mode changed, clearing advertised state");
    advertisedRef.current = {};
    ensureAdvertised();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [cfg.publishTopic, useRos]);

  const sendCommand = (commandName: string, options: { active: boolean }) => {
    console.log("[BehaviorTree] sendCommand called:", commandName, options);
    
    if (!context.publish) {
      console.warn("[BehaviorTree] publish API not available");
      setStatus("发布功能不可用");
      return;
    }

    // Ensure topic is advertised before publishing
    ensureAdvertised();

    // 如果尚未记录为已广告，延迟重试一次
    const mode: "ros" | "custom" = useRos ? "ros" : "custom";
    if (advertisedRef.current.topic !== cfg.publishTopic || advertisedRef.current.mode !== mode) {
      console.warn("[BehaviorTree] Topic not yet advertised (topic match:", 
        advertisedRef.current.topic === cfg.publishTopic, "mode match:", 
        advertisedRef.current.mode === mode, "), retrying...");
      setStatus("正在广告话题，稍后发送...");
      setTimeout(() => {
        ensureAdvertised();
        if (advertisedRef.current.topic === cfg.publishTopic && advertisedRef.current.mode === mode) {
          console.log("[BehaviorTree] Retry: topic now advertised, resending command");
          // 重试发布
          sendCommand(commandName, options);
        } else {
          console.error("[BehaviorTree] Retry failed: topic still not advertised");
          setStatus("广告话题失败");
        }
      }, 200);
      return;
    }

    try {
      // Status: SUCCESS=2 (activate), FAILURE=0 (deactivate)
      const commandStatus = options.active ? 2 : 0; // SUCCESS means activate, FAILURE means deactivate

      const msg = {
        commands: [
          {
            condition_name: commandName,
            status: commandStatus,
          },
        ],
      };

      // Use optional chaining like gimbal panel
      console.log("[BehaviorTree] Publishing message to", cfg.publishTopic, ":", msg);
      context.publish?.(cfg.publishTopic, msg);
      console.log("[BehaviorTree] ✓ Message published");

      const commandLabel =
        BEHAVIOR_COMMANDS.find((cmd) => cmd.name === commandName)?.label ?? commandName;
      setLastSentCommand(`${commandLabel} (${options.active ? "activate" : "deactivate"})`);
      setStatus(`Sent: ${commandLabel}`);

      // 清除状态消息
      setTimeout(() => {
        setStatus(`Connected to: ${cfg.publishTopic}`);
      }, 2000);
    } catch {
      // Silently ignore publish errors (like gimbal panel)
    }
  };

  const buttonStyle: React.CSSProperties = {
    padding: "12px 20px",
    margin: "8px",
    borderRadius: 8,
    border: "2px solid #4a90e2",
    background: "#f0f8ff",
    cursor: "pointer",
    fontWeight: 600,
    fontSize: "14px",
    minWidth: "180px",
    transition: "all 0.2s",
  };

  const buttonHoverStyle: React.CSSProperties = {
    background: "#e0f0ff",
    borderColor: "#2e6bc7",
    transform: "scale(1.02)",
  };

  return (
    <div
      style={{
        padding: "20px",
        display: "flex",
        flexDirection: "column",
        height: "100%",
        overflow: "auto",
      }}
    >
      <div style={{ marginBottom: "20px" }}>
        <h2 style={{ margin: "0 0 10px 0", fontSize: "18px", fontWeight: 600 }}>
          Behavior Tree Controller
        </h2>
        <div style={{ fontSize: "12px", color: "#666", marginBottom: "10px" }}>
          {status} · Data source: {context.dataSourceProfile ?? "unknown"} · Publish: {context.publish ? "Available" : "Unavailable"}
        </div>
        {lastSentCommand && (
          <div style={{ fontSize: "12px", color: "#4a90e2", marginBottom: "10px" }}>
            Last sent: {lastSentCommand}
          </div>
        )}
        <button
          onClick={() => {
            console.log("[BehaviorTree] Manual re-advertise requested");
            advertisedRef.current = {};
            ensureAdvertised();
          }}
          style={{
            padding: "6px 12px",
            fontSize: "11px",
            borderRadius: 4,
            border: "1px solid #ccc",
            background: "#fff",
            cursor: "pointer",
            marginTop: "5px",
          }}
        >
          重新连接话题
        </button>
      </div>

      <div style={{ marginBottom: "20px" }}>
        <h3 style={{ fontSize: "14px", fontWeight: 600, marginBottom: "10px" }}>
          Basic Control
        </h3>
        <div style={{ display: "flex", flexWrap: "wrap", gap: "8px" }}>
          {BEHAVIOR_COMMANDS.slice(0, 5).map((cmd) => (
            <button
              key={cmd.name}
              onClick={() => {
                sendCommand(cmd.name, { active: true });
              }}
              onMouseEnter={(e) => {
                Object.assign(e.currentTarget.style, buttonHoverStyle);
              }}
              onMouseLeave={(e) => {
                Object.assign(e.currentTarget.style, buttonStyle);
              }}
              style={buttonStyle}
              title={cmd.description}
            >
              {cmd.label}
            </button>
          ))}
        </div>
      </div>

      <div style={{ marginBottom: "20px" }}>
        <h3 style={{ fontSize: "14px", fontWeight: 600, marginBottom: "10px" }}>
          Task Control
        </h3>
        <div style={{ display: "flex", flexWrap: "wrap", gap: "8px" }}>
          {BEHAVIOR_COMMANDS.slice(5).map((cmd) => (
            <button
              key={cmd.name}
              onClick={() => {
                sendCommand(cmd.name, { active: true });
              }}
              onMouseEnter={(e) => {
                Object.assign(e.currentTarget.style, buttonHoverStyle);
              }}
              onMouseLeave={(e) => {
                Object.assign(e.currentTarget.style, buttonStyle);
              }}
              style={buttonStyle}
              title={cmd.description}
            >
              {cmd.label}
            </button>
          ))}
        </div>
      </div>

      <div style={{ marginTop: "auto", paddingTop: "20px", fontSize: "11px", color: "#999" }}>
        <div>Topic: {cfg.publishTopic}</div>
        <div>Click buttons to send commands to behavior tree state machine</div>
      </div>
    </div>
  );
}

export function initBehaviorTreeControllerPanel(context: PanelExtensionContext): void {
  const root = createRoot(context.panelElement);
  root.render(<BehaviorTreeControllerPanel context={context} />);
}

// Export component for testing
export { BehaviorTreeControllerPanel };

