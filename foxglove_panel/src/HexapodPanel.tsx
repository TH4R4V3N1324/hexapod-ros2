import {
  Immutable,
  MessageEvent,
  PanelExtensionContext,
  ParameterValue,
  RenderState,
} from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

// Everything here talks to gait_controller (hexapod_gait_controller) through foxglove_bridge
const GAIT_NODE = "hexapod_gait_controller";

// Features gait_controller can switch on and off while running, each with a set_<name> service
// (std_srvs/SetBool) and a parameter of the same name
const FEATURES: { name: string; label: string }[] = [
  { name: "terrain_adaptation", label: "Terrain adaptation (feet)" },
  { name: "terrain_map", label: "Terrain map (depth camera)" },
  { name: "walk_leveling", label: "Level while walking" },
  { name: "tilt_leveling", label: "Level in tilt mode" },
  { name: "tilt_lock_yaw", label: "Lock yaw in tilt mode" },
  { name: "load_balancing", label: "Load balancing" },
];

// Leg numbers in the body's layout, front at the top (leg1 front right ... leg6 front left)
const LEG_LAYOUT: { leg: number; x: number; y: number }[] = [
  { leg: 6, x: 0, y: 0 },
  { leg: 1, x: 2, y: 0 },
  { leg: 5, x: 0, y: 1 },
  { leg: 2, x: 2, y: 1 },
  { leg: 4, x: 0, y: 2 },
  { leg: 3, x: 2, y: 2 },
];

// How often mode, gait and height are re-read from get_capabilities (ms). The gamepad (Start, Back,
// d-pad) and sitting change them without the panel knowing.
const CAPABILITIES_POLL_MS = 2000;
// gait_controller's default depth_topic; the terrain map needs points on it
const DEFAULT_DEPTH_TOPIC = "front_depth/points";

const IMU_TOPIC = "/imu/data";
const POWER_TOPICS = { left: "/power/left", right: "/power/right" };
// behaviour_manager's battery state, one entry in the DiagnosticArrays on /diagnostics
const DIAGNOSTICS_TOPIC = "/diagnostics";
const BATTERY_DIAGNOSTIC = "behaviour_manager: battery";
// servo_monitor's statuses: a summary, and one per servo named after its joint
const SERVO_PREFIX = "servo_monitor: ";
const SERVO_SUMMARY = "servo_monitor: servos";
// servo_monitor publishes once a second; older than this and it has probably stopped
const SERVO_STALE_MS = 5000;
const JOINTS = ["coxa", "femur", "tibia"] as const;
// gait_controller's sit service (std_srvs/SetBool) and whether it's seated (std_msgs/Bool, latched)
const SEATED_TOPIC = "/seated";
const footTopic = (leg: number) => `/foot_force/leg${leg}`;

type Quaternion = { x: number; y: number; z: number; w: number };
type Option = { name: string; id: number };
type Capabilities = {
  modes: Option[];
  gaits: Option[];
  current_mode: number;
  current_gait: number;
  // m, positive: where the body is now, and the range set_height accepts
  current_height: number;
  min_height: number;
  max_height: number;
};
type Battery = { voltage: number; current: number; percentage: number };
type DiagnosticStatus = {
  level: number;
  name: string;
  message: string;
  values: { key: string; value: string }[];
};
// diagnostic_msgs/DiagnosticStatus levels
const DIAG_OK = 0;
const DIAG_WARN = 1;
const DIAG_ERROR = 2;
const DIAG_STALE = 3;
type BatteryStatus = { level: number; message: string; voltage?: string };
type ServoStatus = { level: number; message: string; values: Record<string, string> };
type ServoHealth = { summary: ServoStatus; joints: Record<string, ServoStatus>; receivedAt: number };

const toServoStatus = (s: DiagnosticStatus): ServoStatus => ({
  level: s.level,
  message: s.message,
  values: Object.fromEntries(s.values.map((v) => [v.key, v.value])),
});

function rollPitch(q: Quaternion): { roll: number; pitch: number } {
  const roll = Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (q.w * q.y - q.z * q.x))));
  return { roll, pitch };
}

const degrees = (rad: number) => (rad * 180) / Math.PI;

/** A gait_controller parameter, whatever separator the bridge puts between node and name. */
function gaitParameter(
  parameters: Immutable<Map<string, ParameterValue>> | undefined,
  name: string,
): unknown {
  if (!parameters) {
    return undefined;
  }
  for (const [key, value] of parameters) {
    if (key.includes(GAIT_NODE) && (key.endsWith(`.${name}`) || key.endsWith(`/${name}`))) {
      return value;
    }
  }
  return undefined;
}

function Section({ title, children }: { title: string; children: React.ReactNode }): ReactElement {
  return (
    <section style={{ marginBottom: 14 }}>
      <h4 style={{ margin: "0 0 6px", fontSize: 12, textTransform: "uppercase", opacity: 0.7 }}>
        {title}
      </h4>
      {children}
    </section>
  );
}

/** Horizon that tilts with roll and moves with pitch, as seen from behind the robot. */
function AttitudeIndicator({ roll, pitch }: { roll: number; pitch: number }): ReactElement {
  const size = 72;
  const r = size / 2;
  const shift = Math.max(-r, Math.min(r, (degrees(pitch) / 30) * r));
  return (
    <svg width={size} height={size} viewBox={`${-r} ${-r} ${size} ${size}`} aria-label="attitude">
      <clipPath id="hexapod-attitude-clip">
        <circle r={r - 1} />
      </clipPath>
      <g clipPath="url(#hexapod-attitude-clip)" transform={`rotate(${-degrees(roll)})`}>
        <rect x={-2 * r} y={-2 * r + shift} width={4 * r} height={2 * r} fill="#4a90d9" />
        <rect x={-2 * r} y={shift} width={4 * r} height={2 * r} fill="#9c6b3c" />
        <line x1={-2 * r} x2={2 * r} y1={shift} y2={shift} stroke="white" strokeWidth={1} />
      </g>
      <circle r={r - 1} fill="none" stroke="currentColor" strokeOpacity={0.4} />
      <line x1={-r / 2} x2={r / 2} y1={0} y2={0} stroke="#f5c400" strokeWidth={2} />
    </svg>
  );
}

function HexapodPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [attitude, setAttitude] = useState<{ roll: number; pitch: number } | undefined>();
  const [footForces, setFootForces] = useState<Record<number, number>>({});
  const [power, setPower] = useState<Record<string, Battery>>({});
  const [batteryStatus, setBatteryStatus] = useState<BatteryStatus | undefined>();
  const [servoHealth, setServoHealth] = useState<ServoHealth | undefined>();
  // Ticks once a second so servo health can go stale if servo_monitor stops
  const [clock, setClock] = useState(() => Date.now());
  const [parameters, setParameters] = useState<
    Immutable<Map<string, ParameterValue>> | undefined
  >();
  const [dark, setDark] = useState(true);
  const [capabilities, setCapabilities] = useState<Capabilities | undefined>();
  // Body height (m) shown on the slider; follows current_height except while it's being dragged
  const [height, setHeight] = useState<number | undefined>();
  const dragging = useRef(false);
  const [topics, setTopics] = useState<Set<string>>(new Set());
  const [status, setStatus] = useState<{ text: string; error: boolean } | undefined>();
  // Seated, or asked to sit. seated only changes once a sit finishes or a stand starts, so the
  // button follows the last request until then.
  const [sitting, setSitting] = useState(false);

  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      setParameters(renderState.parameters);
      setDark(renderState.colorScheme !== "light");
      if (renderState.topics) {
        setTopics(new Set(renderState.topics.map((t) => t.name)));
      }
      const frame = renderState.currentFrame;
      if (!frame || frame.length === 0) {
        return;
      }
      const forces: Record<number, number> = {};
      const batteries: Record<string, Battery> = {};
      for (const event of frame as readonly MessageEvent[]) {
        if (event.topic === IMU_TOPIC) {
          const msg = event.message as { orientation: Quaternion };
          setAttitude(rollPitch(msg.orientation));
        } else if (event.topic.startsWith("/foot_force/leg")) {
          const leg = Number(event.topic.slice("/foot_force/leg".length));
          const msg = event.message as { wrench: { force: { z: number } } };
          forces[leg] = msg.wrench.force.z;
        } else if (event.topic === POWER_TOPICS.left || event.topic === POWER_TOPICS.right) {
          const side = event.topic === POWER_TOPICS.left ? "left" : "right";
          batteries[side] = event.message as Battery;
        } else if (event.topic === SEATED_TOPIC) {
          setSitting((event.message as { data: boolean }).data);
        } else if (event.topic === DIAGNOSTICS_TOPIC) {
          const msg = event.message as { status: DiagnosticStatus[] };
          const summary = msg.status.find((s) => s.name === SERVO_SUMMARY);
          if (summary) {
            const joints: Record<string, ServoStatus> = {};
            for (const s of msg.status) {
              if (s.name.startsWith(SERVO_PREFIX) && s.name !== SERVO_SUMMARY) {
                joints[s.name.slice(SERVO_PREFIX.length)] = toServoStatus(s);
              }
            }
            setServoHealth({ summary: toServoStatus(summary), joints, receivedAt: Date.now() });
          }
          const battery = msg.status.find((s) => s.name === BATTERY_DIAGNOSTIC);
          if (battery) {
            setBatteryStatus({
              level: battery.level,
              message: battery.message,
              voltage: battery.values.find((v) => v.key === "voltage")?.value,
            });
          }
        }
      }
      if (Object.keys(forces).length > 0) {
        setFootForces((previous) => ({ ...previous, ...forces }));
      }
      if (Object.keys(batteries).length > 0) {
        setPower((previous) => ({ ...previous, ...batteries }));
      }
    };
    context.watch("currentFrame");
    context.watch("parameters");
    context.watch("colorScheme");
    context.watch("topics");
    context.subscribe([
      { topic: IMU_TOPIC },
      ...LEG_LAYOUT.map(({ leg }) => ({ topic: footTopic(leg) })),
      { topic: POWER_TOPICS.left },
      { topic: POWER_TOPICS.right },
      { topic: DIAGNOSTICS_TOPIC },
      { topic: SEATED_TOPIC },
    ]);
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    const timer = setInterval(() => {
      setClock(Date.now());
    }, 1000);
    return () => {
      clearInterval(timer);
    };
  }, []);

  const call = useCallback(
    async (service: string, request: unknown, description: string): Promise<unknown> => {
      if (!context.callService) {
        setStatus({ text: "This connection can't call services", error: true });
        return undefined;
      }
      try {
        const response = (await context.callService(service, request)) as {
          success?: boolean;
          message?: string;
        };
        if (response.success === false) {
          setStatus({ text: `${description}: ${response.message ?? "refused"}`, error: true });
        } else {
          setStatus({ text: response.message ? `${description}: ${response.message}` : description, error: false });
        }
        return response;
      } catch (error) {
        setStatus({ text: `${description} failed: ${String(error)}`, error: true });
        return undefined;
      }
    },
    [context],
  );

  // quiet: a background poll, which doesn't report failures (the Retry button does)
  const refreshCapabilities = useCallback(
    async (quiet = false) => {
      if (!context.callService) {
        return;
      }
      try {
        const caps = (await context.callService("/get_capabilities", {})) as Capabilities;
        setCapabilities(caps);
        if (!dragging.current) {
          setHeight(caps.current_height);
        }
      } catch (error) {
        if (!quiet) {
          setStatus({ text: `Couldn't reach gait_controller: ${String(error)}`, error: true });
        }
      }
    },
    [context],
  );

  useEffect(() => {
    void refreshCapabilities();
    const timer = setInterval(() => void refreshCapabilities(true), CAPABILITIES_POLL_MS);
    return () => {
      clearInterval(timer);
    };
  }, [refreshCapabilities]);

  // Sitting resets the mode to normal, so re-read as soon as /seated changes
  useEffect(() => {
    void refreshCapabilities(true);
  }, [sitting, refreshCapabilities]);

  const sendHeight = () => {
    dragging.current = false;
    if (height == undefined) {
      return;
    }
    // Sent once the slider is let go, as the feet's height below the hips (negative)
    void call("/set_height", { height: -height }, `Height set to ${(height * 100).toFixed(1)} cm`);
  };

  const depthTopic = String(gaitParameter(parameters, "depth_topic") ?? DEFAULT_DEPTH_TOPIC);
  const hasDepth = topics.has(depthTopic.startsWith("/") ? depthTopic : `/${depthTopic}`);

  const contactThreshold = Number(gaitParameter(parameters, "contact_force_on") ?? 2.0);
  const muted = dark ? "#9aa0a6" : "#5f6368";
  const accent = dark ? "#7cc4ff" : "#1a73e8";
  const warnColor = dark ? "#f5c400" : "#b06000";
  const errorColor = dark ? "#e57373" : "#c5221f";
  const batteryColor = (level: number) =>
    level === DIAG_ERROR ? errorColor : level === DIAG_OK ? undefined : warnColor;
  // behaviour_manager's message is the battery level ("OK", "Low" or "Critical"), then
  // ", no readings from <boards>" if a board has gone quiet (which also makes it a warning)
  const batteryText = (battery: BatteryStatus) => {
    if (battery.level === DIAG_STALE) {
      return `Battery: ${battery.message}`;
    }
    const [state = "", ...rest] = battery.message.split(", ");
    const volts = battery.voltage && battery.voltage !== "n/a" ? ` ${battery.voltage} V` : "";
    const missing = rest.length > 0 ? `. ${rest.join(", ").replace(/^no/, "No")}` : "";
    const advice =
      state === "Critical" ? ". Sitting down: charge or swap the battery" : state === "Low" ? ". Charge soon" : "";
    return `Battery ${state === "OK" ? "OK" : state.toLowerCase()}${volts}${advice}${missing}`;
  };

  const servosStale = servoHealth == undefined || clock - servoHealth.receivedAt > SERVO_STALE_MS;
  const servoSummary = servoHealth && !servosStale ? servoHealth.summary : undefined;
  const servoTitle = (joint: string, servo: ServoStatus | undefined) => {
    if (!servo) {
      return `${joint}: no data`;
    }
    const v = servo.values;
    return [
      `${joint}: ${servo.message}`,
      `${v.temperature ?? "n/a"} °C, ${v.voltage ?? "n/a"} V, ${v.current ?? "n/a"} A`,
      `load ${v.load ?? "n/a"} N·m, errors ${v.errors ?? "n/a"}`,
    ].join("\n");
  };

  return (
    <div style={{ padding: 12, fontFamily: "sans-serif", fontSize: 13, overflowY: "auto", height: "100%" }}>
      {servoSummary && servoSummary.level === DIAG_ERROR && (
        <div
          role="alert"
          style={{
            marginBottom: 12,
            padding: "8px 10px",
            borderRadius: 4,
            border: `2px solid ${errorColor}`,
            color: errorColor,
            fontWeight: "bold",
          }}
        >
          Servos: {servoSummary.message}
        </div>
      )}
      {batteryStatus && batteryStatus.level !== DIAG_OK && (
        <div
          role="alert"
          style={{
            marginBottom: 12,
            padding: "8px 10px",
            borderRadius: 4,
            border: `2px solid ${batteryColor(batteryStatus.level) ?? warnColor}`,
            color: batteryColor(batteryStatus.level),
            fontWeight: "bold",
          }}
        >
          {batteryText(batteryStatus)}
        </div>
      )}

      <Section title="Body">
        {attitude ? (
          <div style={{ display: "flex", alignItems: "center", gap: 14 }}>
            <AttitudeIndicator roll={attitude.roll} pitch={attitude.pitch} />
            <div style={{ fontVariantNumeric: "tabular-nums" }}>
              <div>Roll {degrees(attitude.roll).toFixed(1)}°</div>
              <div>Pitch {degrees(attitude.pitch).toFixed(1)}°</div>
            </div>
          </div>
        ) : (
          <div style={{ color: muted }}>No data on {IMU_TOPIC}</div>
        )}
      </Section>

      <Section title={`Feet (on the ground above ${contactThreshold.toFixed(1)} N)`}>
        <div style={{ display: "grid", gridTemplateColumns: "1fr 24px 1fr", rowGap: 4, maxWidth: 240 }}>
          {LEG_LAYOUT.map(({ leg, x, y }) => {
            const force = footForces[leg];
            const down = force != undefined && force > contactThreshold;
            return (
              <div
                key={leg}
                style={{ gridColumn: x + 1, gridRow: y + 1, display: "flex", alignItems: "center", gap: 6 }}
              >
                <span
                  style={{
                    width: 12,
                    height: 12,
                    borderRadius: 6,
                    border: `2px solid ${accent}`,
                    background: down ? accent : "transparent",
                  }}
                />
                <span style={{ fontVariantNumeric: "tabular-nums" }}>
                  Leg {leg} {force == undefined ? "–" : `${force.toFixed(1)} N`}
                </span>
              </div>
            );
          })}
        </div>
      </Section>

      <Section title="Power">
        <div style={{ marginBottom: 2, color: batteryStatus ? batteryColor(batteryStatus.level) : muted }}>
          {batteryStatus ? batteryText(batteryStatus) : "No battery status from behaviour_manager"}
        </div>
        {(["left", "right"] as const).map((side) => {
          const battery = power[side];
          return (
            <div key={side} style={{ fontVariantNumeric: "tabular-nums" }}>
              {side === "left" ? "Left" : "Right"}:{" "}
              {battery
                ? `${battery.voltage.toFixed(2)} V, ${Math.abs(battery.current).toFixed(2)} A` +
                  (Number.isFinite(battery.percentage) ? `, ${Math.round(battery.percentage * 100)}%` : "")
                : <span style={{ color: muted }}>no data</span>}
            </div>
          );
        })}
      </Section>

      <Section title="Servos">
        {servoSummary ? (
          <>
            <div style={{ marginBottom: 6, color: batteryColor(servoSummary.level === DIAG_STALE ? DIAG_WARN : servoSummary.level) }}>
              {servoSummary.message}
              {servoSummary.values.max_current && servoSummary.values.max_current !== "n/a" && (
                <span style={{ color: muted }}>
                  {" "}
                  · peak {servoSummary.values.max_current} A · min {servoSummary.values.min_voltage} V
                </span>
              )}
            </div>
            {/* Each leg's coxa, femur and tibia temperature, coloured by the servo's level; hover for details */}
            <div style={{ display: "grid", gridTemplateColumns: "1fr 24px 1fr", rowGap: 4, maxWidth: 280 }}>
              {LEG_LAYOUT.map(({ leg, x, y }) => (
                <div
                  key={leg}
                  style={{ gridColumn: x + 1, gridRow: y + 1, display: "flex", gap: 6, fontVariantNumeric: "tabular-nums" }}
                >
                  <span style={{ color: muted }}>L{leg}</span>
                  {JOINTS.map((part) => {
                    const joint = `leg${leg}_${part}_joint`;
                    const servo = servoHealth?.joints[joint];
                    const temperature = servo?.values.temperature;
                    const answering = temperature != undefined && temperature !== "n/a";
                    const color =
                      !servo || !answering ? muted : servo.level === DIAG_STALE ? warnColor : batteryColor(servo.level);
                    return (
                      <span
                        key={part}
                        title={servoTitle(joint, servo)}
                        style={{ color, fontWeight: servo && servo.level !== DIAG_OK ? "bold" : undefined, minWidth: 26 }}
                      >
                        {answering ? `${temperature}°` : servo?.message === "Servo power off" ? "off" : "–"}
                      </span>
                    );
                  })}
                </div>
              ))}
            </div>
            <div style={{ color: muted, fontSize: 11, marginTop: 4 }}>°C for coxa, femur, tibia; hover for details</div>
          </>
        ) : (
          <div style={{ color: muted }}>
            {servoHealth ? "servo_monitor stopped reporting" : "No servo health from servo_monitor"}
          </div>
        )}
      </Section>

      <Section title="Mode and gait">
        {capabilities ? (
          <div style={{ display: "grid", gridTemplateColumns: "auto 1fr", gap: "6px 10px", alignItems: "center" }}>
            <label htmlFor="hexapod-mode">Mode</label>
            <select
              id="hexapod-mode"
              value={capabilities.current_mode}
              onChange={(event) => {
                const mode = Number(event.target.value);
                const name = capabilities.modes.find((m) => m.id === mode)?.name ?? mode;
                void call("/set_mode", { mode }, `Mode set to ${name}`).then(() => refreshCapabilities());
              }}
            >
              {capabilities.modes.map((mode) => (
                <option key={mode.id} value={mode.id}>
                  {mode.name}
                </option>
              ))}
            </select>
            <label htmlFor="hexapod-gait">Gait</label>
            <select
              id="hexapod-gait"
              value={capabilities.current_gait}
              onChange={(event) => {
                const gait = Number(event.target.value);
                const name = capabilities.gaits.find((g) => g.id === gait)?.name ?? gait;
                void call("/set_gait", { gait }, `Gait set to ${name}`).then(() => refreshCapabilities());
              }}
            >
              {capabilities.gaits.map((gait) => (
                <option key={gait.id} value={gait.id}>
                  {gait.name}
                </option>
              ))}
            </select>
            <label htmlFor="hexapod-height">Height</label>
            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <input
                id="hexapod-height"
                type="range"
                min={capabilities.min_height}
                max={capabilities.max_height}
                step={0.005}
                value={height ?? capabilities.current_height}
                onPointerDown={() => {
                  dragging.current = true;
                }}
                onKeyDown={() => {
                  dragging.current = true;
                }}
                onChange={(event) => {
                  setHeight(Number(event.target.value));
                }}
                onPointerUp={sendHeight}
                onKeyUp={sendHeight}
                onPointerCancel={() => {
                  dragging.current = false;
                }}
              />
              <span style={{ fontVariantNumeric: "tabular-nums" }}>
                {((height ?? capabilities.current_height) * 100).toFixed(1)} cm
              </span>
            </div>
            <span>Posture</span>
            <div>
              <button
                onClick={() => {
                  const sit = !sitting;
                  void call("/sit", { data: sit }, sit ? "Sit" : "Stand").then((response) => {
                    if ((response as { success?: boolean } | undefined)?.success === true) {
                      setSitting(sit);
                    }
                  });
                }}
              >
                {sitting ? "Stand up" : "Sit down"}
              </button>
            </div>
          </div>
        ) : (
          <div style={{ color: muted }}>
            Waiting for gait_controller{" "}
            <button onClick={() => void refreshCapabilities()}>Retry</button>
          </div>
        )}
      </Section>

      <Section title="Features">
        {FEATURES.map(({ name, label }) => {
          const value = gaitParameter(parameters, name);
          const known = typeof value === "boolean";
          // The terrain map only does anything with a depth camera publishing points
          const noInput = name === "terrain_map" && !hasDepth;
          return (
            <label key={name} style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 4 }}>
              <input
                type="checkbox"
                checked={value === true}
                disabled={!known || noInput}
                onChange={(event) => {
                  void call(`/set_${name}`, { data: event.target.checked }, label);
                }}
              />
              <span style={{ color: known && !noInput ? undefined : muted }}>
                {label}
                {noInput && ` (nothing on ${depthTopic})`}
              </span>
            </label>
          );
        })}
      </Section>

      {status && (
        <div style={{ color: status.error ? "#e57373" : muted, fontSize: 12 }}>{status.text}</div>
      )}
    </div>
  );
}

export function initHexapodPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<HexapodPanel context={context} />);
  return () => {
    root.unmount();
  };
}
