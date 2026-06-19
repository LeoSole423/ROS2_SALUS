"""Medición de KPIs para validar el `scan_ground_filter` en la rampa.

Replica la metodología de la Etapa A (ver `docs/plan_correccion_puntos_fantasma.md`):

- **FP (falsos positivos)**: celdas ocupadas en el costmap local acumuladas a lo
  largo de la corrida + máximo por frame. En el escenario de rampa
  (`slope_lidar.world`), casi todas las celdas ocupadas provienen del piso leído
  como obstáculo (puntos fantasma). El filtro debe bajarlas drásticamente.
- **Eventos de freno falsos**: frenadas del `collision_monitor` sin obstáculo real,
  detectadas comparando `/cmd_vel` (comandado) contra `/cmd_vel_safe` (tras el
  monitor). Solo se cuentan cuando se comanda avance.

Corre durante `duration_s` y al terminar escribe un JSON con el resumen y apaga el
nodo, para encadenar corridas A/B (flag off vs on) desde un script.
"""

import json
from dataclasses import asdict, dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


@dataclass
class ValidationReport:
    label: str = "run"
    duration_s: float = 0.0
    costmap_frames: int = 0
    fp_accumulated: int = 0
    fp_mean_per_frame: float = 0.0
    fp_max_frame: int = 0
    slowdown_events: int = 0
    stop_events: int = 0


class ValidationAccumulator:
    """Lógica de conteo pura (sin ROS), para poder testearla aislada."""

    def __init__(self, occupied_threshold: int = 100, forward_eps: float = 0.02) -> None:
        self.occupied_threshold = occupied_threshold
        self.forward_eps = forward_eps

        self.costmap_frames = 0
        self.fp_accumulated = 0
        self.fp_max_frame = 0
        self._costmap_grid = None
        self._costmap_width = 0
        self._costmap_height = 0

        self.slowdown_events = 0
        self.stop_events = 0
        self._latest_cmd = 0.0
        self._latest_safe = 0.0
        self._braking = False

    def _record_costmap_frame(self, values: np.ndarray) -> int:
        occ = int(np.count_nonzero(values >= self.occupied_threshold))
        self.fp_accumulated += occ
        self.costmap_frames += 1
        if occ > self.fp_max_frame:
            self.fp_max_frame = occ
        return occ

    def add_costmap(
        self, values: np.ndarray, width: int = 0, height: int = 0
    ) -> int:
        """`values` = data del OccupancyGrid full. Devuelve ocupadas en el frame."""
        data = np.asarray(values, dtype=np.int16)
        if width > 0 and height > 0 and data.size == width * height:
            self._costmap_grid = data.copy()
            self._costmap_width = width
            self._costmap_height = height
        return self._record_costmap_frame(data)

    def add_costmap_update(
        self, x: int, y: int, width: int, height: int, values: np.ndarray
    ) -> int:
        """Aplica un OccupancyGridUpdate y cuenta el grid full resultante."""
        if self._costmap_grid is None:
            return 0
        if width <= 0 or height <= 0:
            return 0
        if x < 0 or y < 0:
            return 0
        if x + width > self._costmap_width or y + height > self._costmap_height:
            return 0

        patch = np.asarray(values, dtype=np.int16)
        if patch.size != width * height:
            return 0

        grid = self._costmap_grid.reshape(self._costmap_height, self._costmap_width)
        grid[y : y + height, x : x + width] = patch.reshape(height, width)
        return self._record_costmap_frame(self._costmap_grid)

    def update_cmd(self, forward: float) -> None:
        self._latest_cmd = forward

    def update_safe(self, forward: float) -> None:
        self._latest_safe = forward
        self._eval_brake()

    def _eval_brake(self) -> None:
        cmd = self._latest_cmd
        safe = self._latest_safe
        if cmd > self.forward_eps:
            is_stop = safe <= self.forward_eps
            is_slow = (not is_stop) and (safe < cmd - self.forward_eps)
            braking = is_stop or is_slow
            # contar solo el flanco de subida de un episodio de frenado
            if braking and not self._braking:
                if is_stop:
                    self.stop_events += 1
                else:
                    self.slowdown_events += 1
            self._braking = braking
        else:
            self._braking = False

    def report(self, label: str, duration_s: float) -> ValidationReport:
        mean = self.fp_accumulated / self.costmap_frames if self.costmap_frames else 0.0
        return ValidationReport(
            label=label,
            duration_s=round(duration_s, 2),
            costmap_frames=self.costmap_frames,
            fp_accumulated=self.fp_accumulated,
            fp_mean_per_frame=round(mean, 2),
            fp_max_frame=self.fp_max_frame,
            slowdown_events=self.slowdown_events,
            stop_events=self.stop_events,
        )


class ScanGroundValidationNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_ground_validation")
        self.declare_parameter("label", "run")
        self.declare_parameter("duration_s", 60.0)
        self.declare_parameter("output_path", "")
        self.declare_parameter("costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("costmap_updates_topic", "/local_costmap/costmap_updates")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_safe_topic", "/cmd_vel_safe")
        self.declare_parameter("occupied_threshold", 100)

        self.label = self._p("label")
        self.duration_s = float(self._p("duration_s"))
        self.output_path = self._p("output_path")

        self.acc = ValidationAccumulator(
            occupied_threshold=int(self._p("occupied_threshold"))
        )

        # El OccupancyGrid full se publica latched (transient_local): hay que
        # suscribirse con la misma durabilidad o, si el validador arranca tarde,
        # nunca llega el grid base y se descartan todos los updates (que son
        # parches relativos). Los updates van por un tópico volátil aparte.
        full_costmap_qos = QoSProfile(depth=1)
        full_costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        full_costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        updates_qos = QoSProfile(depth=10)
        updates_qos.reliability = ReliabilityPolicy.RELIABLE
        updates_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(
            OccupancyGrid, self._p("costmap_topic"), self._on_costmap, full_costmap_qos
        )
        self.create_subscription(
            OccupancyGridUpdate,
            self._p("costmap_updates_topic"),
            self._on_costmap_update,
            updates_qos,
        )
        self.create_subscription(
            Twist, self._p("cmd_vel_topic"), self._on_cmd, 10
        )
        self.create_subscription(
            Twist, self._p("cmd_vel_safe_topic"), self._on_safe, 10
        )

        self._start = self.get_clock().now()
        self._done = False
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f"validación '{self.label}' corriendo {self.duration_s:.0f}s "
            f"(costmap {self._p('costmap_topic')})"
        )

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self.acc.add_costmap(
            np.asarray(msg.data, dtype=np.int16), msg.info.width, msg.info.height
        )

    def _on_costmap_update(self, msg: OccupancyGridUpdate) -> None:
        self.acc.add_costmap_update(
            msg.x, msg.y, msg.width, msg.height, np.asarray(msg.data, dtype=np.int16)
        )

    def _on_cmd(self, msg: Twist) -> None:
        self.acc.update_cmd(msg.linear.x)

    def _on_safe(self, msg: Twist) -> None:
        self.acc.update_safe(msg.linear.x)

    def _tick(self) -> None:
        if self._done:
            return
        elapsed = (self.get_clock().now() - self._start).nanoseconds / 1e9
        if elapsed >= self.duration_s:
            self._done = True
            self._finish(elapsed)

    def _finish(self, elapsed: float) -> None:
        report = self.acc.report(self.label, elapsed)
        as_dict = asdict(report)
        self.get_logger().info("RESULTADO: " + json.dumps(as_dict))
        if self.output_path:
            try:
                with open(self.output_path, "w", encoding="utf-8") as fh:
                    json.dump(as_dict, fh, indent=2)
                self.get_logger().info(f"reporte escrito en {self.output_path}")
            except OSError as exc:
                self.get_logger().error(f"no pude escribir {self.output_path}: {exc}")
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanGroundValidationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
