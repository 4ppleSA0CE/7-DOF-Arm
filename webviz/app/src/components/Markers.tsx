import { useMemo, useRef } from "react";
import { useFrame } from "@react-three/fiber";
import { TransformControls } from "@react-three/drei";
import { Mesh, SphereGeometry, MeshStandardMaterial } from "three";
import type { RobotState, Command } from "../types";

// robot(x,y,z) -> three(x, z, -y); inverse three(x,y,z) -> robot(x, -z, y).
// (the robot Object3D is rotated -90deg about X, so markers placed in world
//  three-space must convert to/from the URDF/robot frame the server speaks.)
const toRobot = (x: number, y: number, z: number) => [x, -z, y];

/**
 * Green target sphere + red obstacle sphere (both draggable) + yellow tip dot.
 * Positions are driven imperatively from the freshest streamed state every
 * frame, except the marker currently being dragged. TransformControls is
 * attached to each mesh via `object=` so the gizmo tracks the moving sphere.
 * On drag-end the new world position is converted to the robot frame and sent.
 */
export function Markers({
  stateRef,
  send,
  obstacleOn,
}: {
  stateRef: React.MutableRefObject<RobotState | null>;
  send: (c: Command) => void;
  obstacleOn: boolean;
}) {
  const targetMesh = useMemo(
    () =>
      new Mesh(
        new SphereGeometry(0.025, 24, 24),
        new MeshStandardMaterial({ color: 0x22cc44, transparent: true, opacity: 0.8 }),
      ),
    [],
  );
  const obstacleMesh = useMemo(
    () => new Mesh(new SphereGeometry(0.06, 24, 24), new MeshStandardMaterial({ color: 0xdd2222 })),
    [],
  );
  // The planner keeps the arm out of a sphere larger than the obstacle geom (see
  // obstacle.radius in webviz/config.yaml). Drawing only the 0.06 m geom makes the
  // arm look like it is swerving around nothing, so show the actual keep-out too.
  // Unit radius, scaled from the streamed value each frame.
  const keepoutMesh = useMemo(
    () =>
      new Mesh(
        new SphereGeometry(1, 24, 24),
        new MeshStandardMaterial({
          color: 0xdd2222,
          transparent: true,
          opacity: 0.12,
          depthWrite: false,
        }),
      ),
    [],
  );
  const tipMesh = useMemo(
    () =>
      new Mesh(
        new SphereGeometry(0.012, 16, 16),
        new MeshStandardMaterial({ color: 0xffd400, emissive: 0xaa8800 }),
      ),
    [],
  );
  const dragging = useRef<"target" | "obstacle" | null>(null);
  // What we last sent, held until the server echoes it back. Releasing the marker the
  // instant the drag ends lets the next frame snap it to the server's STALE position
  // for a few frames -- and for a sub-millimetre drag, which the controller's replan
  // dead-band rejects outright, the ball would snap back and stay there.
  const pending = useRef<{ target?: number[]; obstacle?: number[] }>({});
  const settled = (a: number[] | undefined, b: number[]) =>
    !a || Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < 0.003;

  useFrame(() => {
    const s = stateRef.current;
    if (!s) return;
    if (settled(pending.current.target, s.target)) pending.current.target = undefined;
    if (settled(pending.current.obstacle, s.obstacle)) pending.current.obstacle = undefined;
    if (dragging.current !== "target" && !pending.current.target)
      targetMesh.position.set(s.target[0], s.target[2], -s.target[1]);
    if (dragging.current !== "obstacle" && !pending.current.obstacle)
      obstacleMesh.position.set(s.obstacle[0], s.obstacle[2], -s.obstacle[1]);
    keepoutMesh.position.copy(obstacleMesh.position);
    keepoutMesh.scale.setScalar(s.obstacle_radius);
    tipMesh.position.set(s.tip[0], s.tip[2], -s.tip[1]);
  });

  return (
    <>
      <primitive object={targetMesh} />
      {obstacleOn && <primitive object={obstacleMesh} />}
      {obstacleOn && <primitive object={keepoutMesh} />}
      <primitive object={tipMesh} />
      <TransformControls
        object={targetMesh}
        mode="translate"
        size={0.6}
        onMouseDown={() => (dragging.current = "target")}
        onMouseUp={() => {
          const p = targetMesh.position;
          const t = toRobot(p.x, p.y, p.z);
          pending.current.target = t;
          send({ target: t });
          dragging.current = null;
          // Safety net: if the server never accepts it (a drag below its replan
          // dead-band), stop waiting so the marker resumes tracking the truth.
          setTimeout(() => (pending.current.target = undefined), 1000);
        }}
      />
      {obstacleOn && (
        <TransformControls
          object={obstacleMesh}
          mode="translate"
          size={0.6}
          onMouseDown={() => (dragging.current = "obstacle")}
          onMouseUp={() => {
            const p = obstacleMesh.position;
            const o = toRobot(p.x, p.y, p.z);
            pending.current.obstacle = o;
            send({ obstacle: o });
            dragging.current = null;
            setTimeout(() => (pending.current.obstacle = undefined), 1000);
          }}
        />
      )}
    </>
  );
}
