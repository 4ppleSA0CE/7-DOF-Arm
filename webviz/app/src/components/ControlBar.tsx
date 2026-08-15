import { useEffect, useRef, useState } from "react";
import { Button, Paper, Slider, Stack, Typography, ToggleButton } from "@mui/material";
import type { Command, RobotState } from "../types";

/** Speed presets, mirroring a teach pendant's percentage knob. */
const SPEED_MARKS = [0.25, 0.5, 1.0].map((value) => ({ value }));

export function ControlBar({
  send,
  state,
}: {
  send: (c: Command) => void;
  state: RobotState | null;
}) {
  const obstacleOn = state?.obstacle_on ?? true;
  const sweepOn = state?.sweep_on ?? false;

  // The slider is driven locally while it is being dragged and only follows the
  // server's value otherwise. Binding it straight to the streamed state makes the
  // handle jump back under the cursor, because that state only refreshes at 10 Hz.
  const [speed, setSpeed] = useState(1.0);
  const dragging = useRef(false);
  useEffect(() => {
    if (!dragging.current && state) setSpeed(state.speed_scale);
  }, [state]);
  return (
    <Paper sx={{ p: 1.5 }}>
      <Stack direction="row" gap={1} alignItems="center" flexWrap="wrap">
        <Typography variant="caption" sx={{ opacity: 0.7, flex: 1, minWidth: 170 }}>
          Drag the green ball to retarget{obstacleOn ? " · red ball is the obstacle" : ""}
        </Typography>

        <Stack direction="row" alignItems="center" gap={1.5} sx={{ width: 200 }}>
          <Typography variant="caption" sx={{ opacity: 0.7, whiteSpace: "nowrap" }}>
            speed {Math.round(speed * 100)}%
          </Typography>
          <Slider
            size="small"
            value={speed}
            min={0.1}
            max={1}
            step={0.05}
            marks={SPEED_MARKS}
            onChange={(_, v) => {
              dragging.current = true;
              setSpeed(v as number);
              send({ speed_scale: v as number });
            }}
            onChangeCommitted={() => {
              dragging.current = false;
            }}
          />
        </Stack>

        <ToggleButton
          value="sweep"
          size="small"
          selected={sweepOn}
          color="primary"
          onChange={() => send({ sweep_on: !sweepOn })}
          title="Once the arm arrives, slowly reconfigure the elbow through the null space while the tool stays on the ball"
        >
          Null-space demo {sweepOn ? "on" : "off"}
        </ToggleButton>
        <ToggleButton
          value="obstacle"
          size="small"
          selected={obstacleOn}
          color="error"
          onChange={() => send({ obstacle_on: !obstacleOn })}
        >
          Obstacle {obstacleOn ? "on" : "off"}
        </ToggleButton>
        <Button variant="outlined" size="small" onClick={() => send({ reset: true })}>
          Reset
        </Button>
      </Stack>
    </Paper>
  );
}
