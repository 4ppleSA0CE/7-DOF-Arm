export interface RobotState {
  q: number[];
  tip: number[];
  target: number[];
  obstacle: number[];
  elbow: number[];
  phase: "moving" | "holding" | "stalled" | "unreachable";
  obstacle_on: boolean;
  sweep_on: boolean;
  tip_err_mm: number;
  clearance_m: number;
  obstacle_radius: number; // planner keep-out radius, larger than the drawn geom
  manip: number;
  tau: number[];

  // motion telemetry
  progress: number;          // 0..1 along the current planned move
  tip_speed: number;         // m/s
  joint_speed_max: number;   // rad/s, worst joint
  track_err_rad: number;     // ||q_ref - q||
  speed_scale: number;       // teach-pendant speed override
  eta_s: number;             // seconds left in the current move

  // forward-kinematics readout
  fk_quat: number[]; // [w, x, y, z]

  // live inverse-kinematics check
  q_ik: number[];
  ik_ok: boolean;
  ik_pos_mm: number;
  ik_ori_deg: number;
  ik_joint_dist: number;
}

export type Command =
  | { target: number[] }
  | { obstacle: number[] }
  | { obstacle_on: boolean }
  | { sweep_on: boolean }
  | { speed_scale: number }
  | { reset: true };
