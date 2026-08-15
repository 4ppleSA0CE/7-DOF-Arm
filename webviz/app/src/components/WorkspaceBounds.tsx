// Reachable-workspace envelope: a translucent sphere centered on the shoulder at
// the arm's reach radius, plus a ring on the ground marking the reach footprint.
// Replaces the infinite grid so it is clear where the tool can actually be sent.
// Coordinates are in three-space (the robot Object3D maps robot-z -> three-y, so
// "up" is +y); the shoulder sits ~0.28 m above the base.
export function WorkspaceBounds({ radius = 0.85, shoulderY = 0.28 }: { radius?: number; shoulderY?: number }) {
  return (
    <group>
      <mesh position={[0, shoulderY, 0]}>
        <sphereGeometry args={[radius, 40, 28]} />
        <meshBasicMaterial color="#3a86c8" wireframe transparent opacity={0.1} />
      </mesh>
      <mesh position={[0, shoulderY, 0]}>
        <sphereGeometry args={[radius, 40, 28]} />
        <meshBasicMaterial color="#3a86c8" transparent opacity={0.04} />
      </mesh>
      {/* reach footprint on the ground (three xz-plane) */}
      <mesh rotation={[-Math.PI / 2, 0, 0]}>
        <ringGeometry args={[radius - 0.008, radius, 96]} />
        <meshBasicMaterial color="#4a97d8" transparent opacity={0.6} side={2} />
      </mesh>
    </group>
  );
}
