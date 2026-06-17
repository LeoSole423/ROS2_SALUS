import numpy as np

from navegacion_gps.scan_ground_validation import ValidationAccumulator


def _grid(occupied, total):
    """Devuelve un OccupancyGrid.data sintético con `occupied` celdas a 100."""
    data = np.zeros(total, dtype=np.int16)
    data[:occupied] = 100
    return data


def test_fp_accumulates_and_tracks_max():
    acc = ValidationAccumulator()
    assert acc.add_costmap(_grid(10, 100)) == 10
    assert acc.add_costmap(_grid(25, 100)) == 25
    assert acc.add_costmap(_grid(5, 100)) == 5
    rep = acc.report("x", 3.0)
    assert rep.costmap_frames == 3
    assert rep.fp_accumulated == 40
    assert rep.fp_max_frame == 25
    assert rep.fp_mean_per_frame == round(40 / 3, 2)


def test_costmap_update_counts_full_reconstructed_grid():
    acc = ValidationAccumulator()
    assert acc.add_costmap(np.zeros(9, dtype=np.int16), width=3, height=3) == 0
    assert acc.add_costmap_update(
        x=1,
        y=1,
        width=2,
        height=2,
        values=np.array([100, 100, 0, 100], dtype=np.int16),
    ) == 3
    assert acc.add_costmap_update(
        x=0,
        y=0,
        width=1,
        height=1,
        values=np.array([100], dtype=np.int16),
    ) == 4
    rep = acc.report("updates", 3.0)
    assert rep.costmap_frames == 3
    assert rep.fp_accumulated == 7
    assert rep.fp_max_frame == 4


def test_occupied_threshold_excludes_inflation():
    # celdas a 99 (inflación) no cuentan como ocupadas (umbral 100)
    acc = ValidationAccumulator(occupied_threshold=100)
    data = np.array([100, 100, 99, 50, 0, -1], dtype=np.int16)
    assert acc.add_costmap(data) == 2


def test_no_brake_event_when_safe_follows_cmd():
    acc = ValidationAccumulator()
    for _ in range(5):
        acc.update_cmd(0.5)
        acc.update_safe(0.5)
    assert acc.slowdown_events == 0
    assert acc.stop_events == 0


def test_stop_event_counted_once_per_episode():
    acc = ValidationAccumulator()
    acc.update_cmd(0.5)
    acc.update_safe(0.5)       # avanza
    acc.update_safe(0.0)       # flanco: frenado completo -> 1 stop
    acc.update_safe(0.0)       # sigue frenado -> no recuenta
    acc.update_safe(0.5)       # recupera
    acc.update_safe(0.0)       # nuevo episodio -> 2do stop
    assert acc.stop_events == 2
    assert acc.slowdown_events == 0


def test_slowdown_event_distinguished_from_stop():
    acc = ValidationAccumulator()
    acc.update_cmd(0.5)
    acc.update_safe(0.25)      # reduce sin frenar del todo -> slowdown
    assert acc.slowdown_events == 1
    assert acc.stop_events == 0


def test_no_event_when_not_commanding_forward():
    acc = ValidationAccumulator()
    acc.update_cmd(0.0)
    acc.update_safe(0.0)       # quieto y frenado: no es freno falso
    assert acc.stop_events == 0
    assert acc.slowdown_events == 0
