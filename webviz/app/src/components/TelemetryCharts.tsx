import { useEffect, useRef, useState } from "react";
import { Line } from "react-chartjs-2";
import {
  Chart,
  LineElement,
  PointElement,
  LinearScale,
  CategoryScale,
  Tooltip,
} from "chart.js";
import { Paper, Typography } from "@mui/material";
import type { RobotState } from "../types";

Chart.register(LineElement, PointElement, LinearScale, CategoryScale, Tooltip);

const N = 150;

export function TelemetryCharts({ state }: { state: RobotState | null }) {
  // Append in an effect, not during render. Pushing during render is a side effect,
  // so React.StrictMode's double-render recorded every sample twice and the traces
  // scrolled at 2x speed in development.
  const hist = useRef<{ err: number[]; clr: number[] }>({ err: [], clr: [] }).current;
  const [, bump] = useState(0);
  useEffect(() => {
    if (!state) return;
    hist.err.push(state.tip_err_mm);
    if (hist.err.length > N) hist.err.shift();
    hist.clr.push(state.clearance_m);
    if (hist.clr.length > N) hist.clr.shift();
    bump((n) => n + 1);
  }, [state, hist]);
  const err = hist.err;
  const clr = hist.clr;

  const opts = {
    animation: false as const,
    responsive: true,
    scales: { x: { display: false }, y: { ticks: { color: "#888" } } },
    plugins: { legend: { display: false } },
  };
  const mk = (data: number[], color: string) => ({
    labels: data.map((_, i) => i),
    datasets: [{ data: [...data], borderColor: color, pointRadius: 0, borderWidth: 2 }],
  });

  return (
    <Paper sx={{ p: 1.5 }}>
      <Typography variant="caption" sx={{ opacity: 0.7 }}>
        Tip error (mm)
      </Typography>
      <Line data={mk(err, "#22cc44")} options={opts} />
      <Typography variant="caption" sx={{ opacity: 0.7, mt: 1, display: "block" }}>
        Elbow clearance (m)
      </Typography>
      <Line data={mk(clr, "#dd8800")} options={opts} />
    </Paper>
  );
}
