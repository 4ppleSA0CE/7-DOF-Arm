import { useEffect, useRef, useState, useCallback } from "react";
import type { RobotState, Command } from "../types";

/**
 * Connects to the MuJoCo sim server. Exposes:
 *   stateRef  - freshest state, read inside useFrame (no re-render)
 *   state     - throttled to 10 Hz for React UI (charts, panels)
 *   connected - socket status
 *   send      - push a command to the server
 * Auto-reconnects on drop.
 */
// 8765 is Foxglove's default WebSocket port, so the sim server stays clear of it.
// Override with VITE_WS_URL (see webviz/app/.env) if you change server.port in
// webviz/config.yaml.
const DEFAULT_WS_URL = import.meta.env.VITE_WS_URL ?? "ws://localhost:8770";

export function useRobotSocket(url = DEFAULT_WS_URL) {
  const wsRef = useRef<WebSocket | null>(null);
  const stateRef = useRef<RobotState | null>(null);
  const [state, setState] = useState<RobotState | null>(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    let alive = true;
    let timer: ReturnType<typeof setTimeout>;
    const connect = () => {
      const ws = new WebSocket(url);
      wsRef.current = ws;
      ws.onopen = () => setConnected(true);
      ws.onclose = () => {
        setConnected(false);
        if (alive) timer = setTimeout(connect, 1000);
      };
      ws.onmessage = (e) => {
        stateRef.current = JSON.parse(e.data) as RobotState;
      };
    };
    connect();
    const ui = setInterval(() => {
      if (stateRef.current) setState(stateRef.current);
    }, 100);
    return () => {
      alive = false;
      clearTimeout(timer);
      clearInterval(ui);
      wsRef.current?.close();
    };
  }, [url]);

  const send = useCallback((cmd: Command) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(cmd));
    }
  }, []);

  return { stateRef, state, connected, send };
}
