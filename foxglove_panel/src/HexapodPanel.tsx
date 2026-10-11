import {
  Immutable,
  MessageEvent,
  PanelExtensionContext,
  ParameterValue,
  RenderState,
} from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useState } from "react";
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

// Body height (m) the slider allows; set_height has no limits of its own
const HEIGHT_MIN = 0.08;
const HEIGHT_MAX = 0.15;
const HEIGHT_DEFAULT = 0.12;

const IMU_TOPIC = "/imu/data";
const POWER_TOPICS = { left: "/power/left", right: "/power/right" };
const footTopic = (leg: number) => `/foot_force/leg${leg}`;

type Quaternion = { x: number; y: number; z: number; w: number };
type Option = { name: string; id: number };
type Capabilities = {
  modes: Option[];
  gaits: Option[];
  current_mode: number;
  current_gait: number;
};
type Battery = { voltage: number; current: number; percentage: number };

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
  const [parameters, setParameters] = useState<
    Immutable<Map<string, ParameterValue>> | undefined
  >();
  const [dark, setDark] = useState(true);
  const [capabilities, setCapabilities] = useState<Capabilities | undefined>();
  const [height, setHeight] = useState(HEIGHT_DEFAULT);
  const [status, setStatus] = useState<{ text: string; error: boolean } | undefined>();

  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      setParameters(renderState.parameters);
      setDark(renderState.colorScheme !== "light");
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
    context.subscribe([
      { topic: IMU_TOPIC },
      ...LEG_LAYOUT.map(({ leg }) => ({ topic: footTopic(leg) })),
      { topic: POWER_TOPICS.left },
      { topic: POWER_TOPICS.right },
    ]);
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

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

  const refreshCapabilities = useCallback(async () => {
    if (!context.callService) {
      return;
    }
    try {
      setCapabilities((await context.callService("/get_capabilities", {})) as Capabilities);
    } catch (error) {
      setStatus({ text: `Couldn't reach gait_controller: ${String(error)}`, error: true });
    }
  }, [context]);

  useEffect(() => {
    void refreshCapabilities();
  }, [refreshCapabilities]);

  const contactThreshold = Number(gaitParameter(parameters, "contact_force_on") ?? 2.0);
  const muted = dark ? "#9aa0a6" : "#5f6368";
  const accent = dark ? "#7cc4ff" : "#1a73e8";

  return (
    <div style={{ padding: 12, fontFamily: "sans-serif", fontSize: 13, overflowY: "auto", height: "100%" }}>
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
                void call("/set_mode", { mode }, `Mode set to ${name}`).then(refreshCapabilities);
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
                void call("/set_gait", { gait }, `Gait set to ${name}`).then(refreshCapabilities);
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
                min={HEIGHT_MIN}
                max={HEIGHT_MAX}
                step={0.005}
                value={height}
                onChange={(event) => {
                  setHeight(Number(event.target.value));
                }}
                // Sent once the slider is let go, as the feet's height below the hips (negative)
                onPointerUp={() => void call("/set_height", { height: -height }, `Height set to ${(height * 100).toFixed(1)} cm`)}
                onKeyUp={() => void call("/set_height", { height: -height }, `Height set to ${(height * 100).toFixed(1)} cm`)}
              />
              <span style={{ fontVariantNumeric: "tabular-nums" }}>{(height * 100).toFixed(1)} cm</span>
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
          return (
            <label key={name} style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 4 }}>
              <input
                type="checkbox"
                checked={value === true}
                disabled={!known}
                onChange={(event) => {
                  void call(`/set_${name}`, { data: event.target.checked }, label);
                }}
              />
              <span style={{ color: known ? undefined : muted }}>{label}</span>
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
