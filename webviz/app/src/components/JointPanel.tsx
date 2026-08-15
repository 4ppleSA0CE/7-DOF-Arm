import { Paper, Typography, Stack, LinearProgress } from "@mui/material";
import type { RobotState } from "../types";

// continuous joints wrap; show angle normalized into [-pi, pi] for the bar.
const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

function JointRow({ i, q }: { i: number; q: number }) {
  const pct = ((wrap(q) + Math.PI) / (2 * Math.PI)) * 100;
  return (
    <Stack direction="row" alignItems="center" gap={1}>
      <Typography variant="caption" sx={{ width: 56, opacity: 0.7 }}>
        joint_{i + 1}
      </Typography>
      <div style={{ flex: 1 }}>
        <LinearProgress variant="determinate" value={pct} />
      </div>
      <Typography
        variant="caption"
        sx={{ width: 64, textAlign: "right", fontVariantNumeric: "tabular-nums" }}
      >
        {q.toFixed(3)}
      </Typography>
    </Stack>
  );
}

export function JointPanel({ state }: { state: RobotState | null }) {
  return (
    <Paper sx={{ p: 1.5 }}>
      <Typography variant="subtitle2" sx={{ mb: 1, opacity: 0.7 }}>
        Joint angles (rad)
      </Typography>
      <Stack gap={0.75}>
        {(state?.q ?? new Array(7).fill(0)).map((q, i) => (
          <JointRow key={i} i={i} q={q} />
        ))}
      </Stack>
    </Paper>
  );
}
