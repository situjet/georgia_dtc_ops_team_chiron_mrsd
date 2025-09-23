# PX4/MAVROS 起飞测试手册（SITL → 台架 → 外场）

> Purpose: Compile a practical, safe, and repeatable takeoff-testing flow for PX4 + MAVROS + behavior_governor. Stages: **SITL → Bench (props off) → Field (props on)**. 适用于 ROS 2 + MAVROS2。

---

## 0. 环境与准备（Environment & Prep）
- **软件**：PX4 ≥ v1.13（或你项目版本）、MAVROS2、ROS 2 Humble/ Iron（与你环境一致）、行为治理节点 `behavior_governor`、Python 3 运行脚本。
- **硬件**：飞控（PX4 支持）、遥控器（可接管）、GNSS/RTK（户外）、电源与螺旋桨（外场才装桨）。
- **安全**：室内严禁装桨；台架阶段螺旋桨**必须卸下**；外场前设置围挡、观察员、急停预案。
- **参数快照**：测试前后导出 `QGC → Parameters → Save` 以便复现与回退。

---

## A) Pre‑flight Checklist（通用，三阶段皆适用）

### A1. Bringup 顺序（Bringup Order）
- [ ] **PX4**（SITL 或真机）已启动且在发布话题。
- [ ] **MAVROS** 已连接 → `ros2 topic echo /mavros/state` 出现 `connected: True`。
- [ ] **behavior_governor** 节点已运行。

### A2. GPS / Home 就绪（GPS/Home Readiness）
- [ ] `ros2 topic echo /mavros/global_position/global`，检查 `status.status >= 0`。
- [ ] `ros2 topic echo /mavros/home_position/home`，至少收到**一条**消息（Home 已设定）。

### A3. Failsafe/模式合理性（Failsafe Params Sanity）
- [ ] **SITL**：默认可用。
- [ ] **室内台架**：无 GPS 时**不要**尝试 `AUTO.TAKEOFF`（需 GPS）。改用 SITL 或移步室外，或改 OFFBOARD + 本地定位（VIO/LPS）。

### A4. 起飞高度（Takeoff Altitude）
- [ ] `MIS_TAKEOFF_ALT`（相对 Home，高度：米）设置为目标，如 **3–5 m**（外场初测保守）。

> 注：PX4 自动起飞使用 `MIS_TAKEOFF_ALT`。

---

## B) 阶段 1 — SITL（强烈建议先做）
**目标**：在仿真中闭环验证（governor → MAVROS → PX4）链路与起飞流程，安全、可复现。

### B1. 参考 Bringup（可选流程之一）
```bash
# 1) 启动 PX4 SITL（任选 jmavsim / gazebo-classic / gz）
make px4_sitl jmavsim
# or
# make px4_sitl gazebo-classic
# make px4_sitl gz

# 2) 启动 MAVROS2（按你的 SITL UDP 端口调整 fcu_url）
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557

# 3) 启动 behavior_governor（你的节点）
ros2 run your_pkg behavior_governor

# 4) 可选监控
ros2 topic echo /behavior_governor/status
ros2 topic echo /mavros/global_position/rel_alt
ros2 topic echo /mavros/state
```

### B2. 测试步骤（Test Procedure）
```bash
# 运行起飞测试器（示例）
ros2 run your_pkg takeoff_tester.py
```
**观察日志**：连接 → GPS/Home 就绪 → 解锁（arm）→ 切到 `AUTO.TAKEOFF` → 爬升。

**验收标准**：60 s 内，相对高度 `rel_alt` ≥ `MIS_TAKEOFF_ALT` 的 **~80%**。

**可选记录**：
```bash
ros2 bag record \
  /mavros/state \
  /mavros/global_position/rel_alt \
  /behavior_governor/status
```

### B3. 失败排查（If it fails）
- [ ] 使用 `--direct-mavros` 直连 MAVROS 复测，区分问题在 governor 逻辑 vs. PX4/MAVROS。
- [ ] 观察 `/mavros/state.mode` 是否切换到 `AUTO.TAKEOFF`。
- [ ] 若模式未切换，复核：GPS/Home、已解锁、无 RC 在 Manual 干扰。

---

## C) 阶段 2 — 真实硬件台架（**卸桨**）
**目标**：在真机上验证指令链路与模式切换，但不产生升力。

### C1. 条件
- [ ] 户外、GPS 可用（依然**卸桨**）。
- [ ] 上电、Bringup 顺序与 A 节一致。

### C2. 预期现象
- 连接 → 解锁 → 切换至 `AUTO.TAKEOFF` → 电机**可能短暂转动**，但因无桨无法升空。

### C3. 安全中止
- 优先通过 governor 发送 **land** 指令或等待控制流程结束。
- **不要**在转速较高时强制断电/解锁，避免损伤电调/电机。

### C4. 室内无 GPS 的替代
- 不要用 `AUTO.TAKEOFF`。
- 选项：切换 **OFFBOARD** + 本地定位（VIO/LPS），这是一条**不同控制路径**（需独立验证）。

---

## D) 阶段 3 — 可控外场（**装桨**）
**目标**：真实环境自动起飞到目标高度，并稳定悬停。

### D1. 飞行前设置
- [ ] `MIS_TAKEOFF_ALT`：保守（3–5 m）。
- [ ] Geofence 与 RTL：符合空域与安全策略（失联/低电返航）。
- [ ] 起飞点清场，站位机尾后侧；遥控器置于 **Position/Loiter**，随时可接管。

### D2. 执行
- 运行测试器 → 确认解锁、模式切换、爬升 → 达标高度稳定悬停。

### D3. 收尾
- 通过 governor 下达 loiter / land，或用 RC 接管并降落。

---

## 常见问题速查（Troubleshooting Quick Matrix）
| 现象 | 可能原因 | 快速排查 |
|---|---|---|
| `connected: False` | MAVROS fcu_url/端口不匹配；PX4 未起 | 核对 SITL 端口；`uorb top`/`listener vehicle_status`；重启顺序 |
| 不能切到 `AUTO.TAKEOFF` | 未解锁；GPS/Home 未就绪；RC 在 Manual | `ros2 topic echo /mavros/state` 看 `armed/mode`；检查 `/mavros/home_position/home` 是否发布 |
| 起飞缓慢/不到高度 | `MIS_TAKEOFF_ALT` 太高；气象/模型限制 | 先将高度设为 3–5 m；看 `rel_alt` 曲线与 60 s 验收窗 |
| 室内台架电机狂转 | 误触 `AUTO.TAKEOFF` 或模式未锁定 | 室内禁用 AUTO；改 OFFBOARD + VIO/LPS；必要时上电即禁止解锁 |
| 反复失败难定位 | governor 逻辑/参数问题 | 用 `--direct-mavros` 旁路 governor；包录制复盘 |

---

## 关键参数 & 话题（Key Params & Topics）
- **PX4**：`MIS_TAKEOFF_ALT`（m, relative）；RTL/geofence（根据空域配置）。
- **MAVROS**（ROS 2）：
  - `/mavros/state`（连接、解锁、模式）
  - `/mavros/global_position/global`（含 `status.status`）
  - `/mavros/home_position/home`（Home ready 判据）
  - `/mavros/global_position/rel_alt`（相对高度）
- **Governor**：`/behavior_governor/status`（状态与事件）

---

## 建议的测试记录模板（每轮实验都填）
- 日期 / 机型 / 固件版本：
- 场景（SITL / 台架 / 外场）：
- 关键参数（`MIS_TAKEOFF_ALT`，RTL，Geofence）：
- 流水（Bringup 时间、解锁时间、模式切换时间）：
- 结果（峰值相对高、达标用时）：
- 异常与处置：
- 下次改进项：

---

## 附：最小化命令速查（Cheat‑Sheet）
```bash
# SITL 启动（示例 jmavsim）
make px4_sitl jmavsim

# MAVROS2（按需改端口）
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557

# Governor & 监控
ros2 run your_pkg behavior_governor
ros2 topic echo /mavros/state

# 起飞测试脚本（示例）
ros2 run your_pkg takeoff_tester.py

# 录包
ros2 bag record /mavros/state /mavros/global_position/rel_alt /behavior_governor/status
```

> 提醒：每次推送新任务前先清理旧任务（若使用任务航线）：`/mavros/mission/clear`；允许未解锁下推送任务（PX4/MAVROS 支持）。室内/无 GPS 场景不要做 AUTO.TAKEOFF，改用 OFFBOARD + 本地定位路径。

