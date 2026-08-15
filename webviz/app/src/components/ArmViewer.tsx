import { useEffect, useState } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import URDFLoader from "urdf-loader";
import type { URDFRobot } from "urdf-loader";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { LoadingManager, MeshStandardMaterial, Mesh } from "three";
import type { RobotState } from "../types";

const JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"];

/**
 * Loads j2s7s300.urdf via urdf-loader and applies streamed joint angles each
 * frame. The robot is rotated -90deg about X to map URDF z-up into three y-up.
 */
export function ArmViewer({
  stateRef,
}: {
  stateRef: React.MutableRefObject<RobotState | null>;
}) {
  const [robot, setRobot] = useState<URDFRobot | null>(null);
  const { scene } = useThree();

  useEffect(() => {
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);
    loader.packages = { kinova_gen2_description: "/models" };
    loader.loadMeshCb = (path, mgr, done) => {
      new STLLoader(mgr).load(path, (geom) => {
        const mesh = new Mesh(
          geom,
          new MeshStandardMaterial({ color: 0xcad0ee, metalness: 0.3, roughness: 0.6 }),
        );
        done(mesh);
      });
    };
    loader.load("/models/j2s7s300.urdf", (r) => {
      r.rotation.x = -Math.PI / 2; // URDF z-up -> three y-up
      setRobot(r);
    });
  }, []);

  useEffect(() => {
    if (robot) scene.add(robot);
    return () => {
      if (robot) scene.remove(robot);
    };
  }, [robot, scene]);

  useFrame(() => {
    const s = stateRef.current;
    if (!robot || !s) return;
    JOINTS.forEach((j, i) => robot.setJointValue(j, s.q[i]));
  });

  return null;
}
