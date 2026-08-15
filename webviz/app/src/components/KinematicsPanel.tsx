import { Paper, Stack, Typography, Chip, Divider } from "@mui/material";
import type { RobotState } from "../types";

const f = (x: number, n = 3) => (x ?? 0).toFixed(n);

function Row({ label, value, mono = true }: { label: string; value: string; mono?: boolean }) {
  return (
    <Stack direction="row" justifyContent="space-between" alignItems="baseline">
      <Typography variant="caption" sx={{ opacity: 0.6 }}>
        {label}
      </Typography>
      <Typography
        variant="caption"
        sx={{ fontVariantNumeric: "tabular-nums", fontFamily: mono ? "monospace" : undefined }}
      >
        {value}
      </Typography>
    </Stack>
  );
}

// Live proof that our from-scratch FK and IK work: FK maps the current joints to
// the tool pose; IK inverts that pose back to a joint solution (residual ~0), and
// the recovered posture generally differs from the real arm -> redundancy.
export function KinematicsPanel({ state }: { state: RobotState | null }) {
  const s = state;
  const tip = s?.tip ?? [0, 0, 0];
  const quat = s?.fk_quat ?? [1, 0, 0, 0];
  const ikOk = !!s?.ik_ok && (s?.ik_pos_mm ?? 9) < 0.1 && (s?.ik_ori_deg ?? 9) < 0.01;

  return (
    <Paper sx={{ p: 1.5 }}>
      {/* Forward kinematics */}
      <Typography variant="subtitle2" sx={{ opacity: 0.75, mb: 0.5 }}>
        Forward kinematics&nbsp;
        <Typography component="span" variant="caption" sx={{ opacity: 0.5 }}>
          q → tool pose
        </Typography>
      </Typography>
      <Stack gap={0.4}>
        <Row label="tool x / y / z (m)" value={`${f(tip[0])}  ${f(tip[1])}  ${f(tip[2])}`} />
        <Row
          label="quat w / x / y / z"
          value={`${f(quat[0], 2)} ${f(quat[1], 2)} ${f(quat[2], 2)} ${f(quat[3], 2)}`}
        />
        <Row label="manipulability w" value={f(s?.manip ?? 0, 4)} />
      </Stack>

      <Divider sx={{ my: 1.25 }} />

      {/* Inverse kinematics */}
      <Stack direction="row" alignItems="center" gap={1} sx={{ mb: 0.5 }}>
        <Typography variant="subtitle2" sx={{ opacity: 0.75 }}>
          Inverse kinematics&nbsp;
          <Typography component="span" variant="caption" sx={{ opacity: 0.5 }}>
            tool pose → q
          </Typography>
        </Typography>
        <div style={{ flex: 1 }} />
        <Chip
          size="small"
          label={ikOk ? "✓ inverts FK" : s?.ik_ok ? "converging" : "no solution"}
          color={ikOk ? "success" : "warning"}
          variant="outlined"
        />
      </Stack>
      <Stack gap={0.4}>
        <Row label="position residual" value={`${f(s?.ik_pos_mm ?? 0, 4)} mm`} />
        <Row label="orientation residual" value={`${f(s?.ik_ori_deg ?? 0, 4)}°`} />
        <Row label="recovered posture Δ" value={`${f(s?.ik_joint_dist ?? 0, 2)} rad`} />
      </Stack>
      <Typography variant="caption" sx={{ opacity: 0.5, display: "block", mt: 0.75 }}>
        IK reconstructs a valid q for the tool pose (residual ≈ 0). The Δ is how far that
        posture is from the real arm — same pose, different joints: the redundant DOF.
      </Typography>
    </Paper>
  );
}
