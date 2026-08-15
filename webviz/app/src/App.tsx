import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid } from "@react-three/drei";
import { Stack, Typography } from "@mui/material";
import { useRobotSocket } from "./hooks/useRobotSocket";
import { ArmViewer } from "./components/ArmViewer";
import { Markers } from "./components/Markers";
import { WorkspaceBounds } from "./components/WorkspaceBounds";
import { PhaseBanner } from "./components/PhaseBanner";
import { ControlBar } from "./components/ControlBar";
import { KinematicsPanel } from "./components/KinematicsPanel";
import { JointPanel } from "./components/JointPanel";
import { TelemetryCharts } from "./components/TelemetryCharts";

export default function App() {
  const { stateRef, state, connected, send } = useRobotSocket();
  return (
    <div style={{ display: "flex", height: "100vh" }}>
      <div style={{ flex: 1, position: "relative" }}>
        <Canvas camera={{ position: [1.1, 1.0, 1.1], fov: 50 }} shadows>
          <ambientLight intensity={0.6} />
          <directionalLight position={[2, 3, 2]} intensity={1.2} castShadow />
          <Grid
            args={[1.8, 1.8]}
            cellColor="#2a2a2a"
            sectionColor="#3a3a3a"
            fadeDistance={3.5}
          />
          <WorkspaceBounds />
          <ArmViewer stateRef={stateRef} />
          <Markers stateRef={stateRef} send={send} obstacleOn={state?.obstacle_on ?? true} />
          <OrbitControls makeDefault target={[0, 0.4, 0]} />
        </Canvas>
        <Typography
          variant="overline"
          sx={{ position: "absolute", top: 8, left: 16, opacity: 0.6 }}
        >
          Kinova Gen2 (j2s7s300) — drag the green ball to retarget, red ball to obstruct
        </Typography>
      </div>
      <Stack sx={{ width: 380, p: 2, gap: 2, overflowY: "auto", bgcolor: "background.default" }}>
        <PhaseBanner state={state} connected={connected} />
        <ControlBar send={send} state={state} />
        <KinematicsPanel state={state} />
        <JointPanel state={state} />
        <TelemetryCharts state={state} />
      </Stack>
    </div>
  );
}
