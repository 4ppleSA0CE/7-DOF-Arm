import { Paper, Stack, Typography } from "@mui/material";
import type { RobotState } from "../types";

const LABEL: Record<RobotState["phase"], string> = {
  moving: "MOVING",
  holding: "HOLDING",
  stalled: "BLOCKED",
  unreachable: "OUT OF REACH",
};

const COLOR: Record<RobotState["phase"], string> = {
  moving: "#4da3ff",
  holding: "#22cc44",
  stalled: "#ffaa22",
  unreachable: "#ffaa22",
};

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div>
      <Typography variant="caption" sx={{ opacity: 0.6, display: "block" }}>
        {label}
      </Typography>
      <Typography variant="h6" sx={{ fontVariantNumeric: "tabular-nums" }}>
        {value}
      </Typography>
    </div>
  );
}

export function PhaseBanner({
  state,
  connected,
}: {
  state: RobotState | null;
  connected: boolean;
}) {
  return (
    <Paper sx={{ p: 2 }}>
      <Stack direction="row" alignItems="center" gap={1} sx={{ mb: 1 }}>
        <div
          style={{
            width: 10,
            height: 10,
            borderRadius: "50%",
            background: connected ? "#22cc44" : "#dd2222",
          }}
        />
        <Typography variant="subtitle2" sx={{ opacity: 0.7 }}>
          {connected ? "live" : "disconnected"}
        </Typography>
        <div style={{ flex: 1 }} />
        <Typography
          variant="subtitle1"
          fontWeight={700}
          letterSpacing={1}
          sx={{ color: state ? COLOR[state.phase] : undefined }}
        >
          {state ? LABEL[state.phase] : "—"}
        </Typography>
      </Stack>
      <Stack direction="row" justifyContent="space-between">
        <Stat label="tip error" value={state ? `${state.tip_err_mm.toFixed(1)} mm` : "—"} />
        <Stat label="tip speed" value={state ? `${state.tip_speed.toFixed(3)} m/s` : "—"} />
        <Stat
          label="max joint rate"
          value={state ? `${state.joint_speed_max.toFixed(2)} rad/s` : "—"}
        />
        <Stat label="elbow clearance" value={state ? `${state.clearance_m.toFixed(3)} m` : "—"} />
        <Stat label="manipulability" value={state ? state.manip.toFixed(3) : "—"} />
      </Stack>
      {state && state.phase === "moving" && (
        <Stack direction="row" alignItems="center" gap={1} sx={{ mt: 1.5 }}>
          <div
            style={{
              flex: 1,
              height: 4,
              borderRadius: 2,
              background: "rgba(255,255,255,0.12)",
              overflow: "hidden",
            }}
          >
            <div
              style={{
                width: `${Math.round(state.progress * 100)}%`,
                height: "100%",
                background: COLOR.moving,
              }}
            />
          </div>
          <Typography variant="caption" sx={{ opacity: 0.6, fontVariantNumeric: "tabular-nums" }}>
            {state.eta_s.toFixed(1)}s left
          </Typography>
        </Stack>
      )}
    </Paper>
  );
}
