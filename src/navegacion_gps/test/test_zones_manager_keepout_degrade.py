import cv2
import numpy as np
import yaml

from navegacion_gps.zones_manager import (
    ZonesManagerNode,
    build_scale_mask_yaml_data,
    compose_keepout_cost_mask,
    cost_mask_to_scale_image,
)


def test_mask_files_are_published_from_same_directory_temporaries(
    tmp_path, monkeypatch
):
    node = object.__new__(ZonesManagerNode)
    node.mask_image_file = tmp_path / "keepout_mask.pgm"
    node.mask_yaml_file = tmp_path / "keepout_mask.yaml"
    node.mask_resolution = 0.1
    node.mask_origin_x = -150.0
    node.mask_origin_y = -150.0

    real_imwrite = cv2.imwrite
    written_paths = []

    def recording_imwrite(path, image):
        written_paths.append(path)
        assert path != str(node.mask_image_file)
        return real_imwrite(path, image)

    monkeypatch.setattr(cv2, "imwrite", recording_imwrite)
    image = np.array([[255, 128], [0, 255]], dtype=np.uint8)

    ok, error = node._write_mask_files(image)

    assert ok is True
    assert error == ""
    assert written_paths
    assert np.array_equal(cv2.imread(str(node.mask_image_file), 0), image)
    assert yaml.safe_load(node.mask_yaml_file.read_text(encoding="utf-8"))[
        "image"
    ] == "keepout_mask.pgm"
    assert sorted(path.name for path in tmp_path.iterdir()) == [
        "keepout_mask.pgm",
        "keepout_mask.yaml",
    ]


def test_compose_keepout_cost_mask_disabled_matches_binary_baseline():
    core_binary = np.array(
        [
            [0, 1, 0],
            [1, 1, 1],
            [0, 1, 0],
        ],
        dtype=np.uint8,
    )
    cost_mask = compose_keepout_cost_mask(
        core_binary_mask=core_binary,
        resolution=0.1,
        degrade_enabled=False,
        degrade_radius_m=2.0,
        degrade_edge_cost=12,
        degrade_min_cost=1,
        degrade_use_l2=True,
    )
    expected = np.where(core_binary > 0, 100, 0).astype(np.uint8)
    assert np.array_equal(cost_mask, expected)


def test_compose_keepout_cost_mask_enabled_generates_halo():
    core_binary = np.zeros((81, 81), dtype=np.uint8)
    core_binary[40, 40] = 1

    cost_mask = compose_keepout_cost_mask(
        core_binary_mask=core_binary,
        resolution=0.1,
        degrade_enabled=True,
        degrade_radius_m=2.0,
        degrade_edge_cost=12,
        degrade_min_cost=1,
        degrade_use_l2=True,
    )

    assert int(cost_mask[40, 40]) == 100
    assert int(np.max(cost_mask)) == 100
    assert np.any((cost_mask > 0) & (cost_mask < 100))
    assert int(cost_mask[0, 0]) == 0


def test_compose_keepout_cost_mask_zero_radius_generates_no_halo():
    core_binary = np.zeros((41, 41), dtype=np.uint8)
    core_binary[20, 20] = 1

    cost_mask = compose_keepout_cost_mask(
        core_binary_mask=core_binary,
        resolution=0.1,
        degrade_enabled=True,
        degrade_radius_m=0.0,
        degrade_edge_cost=12,
        degrade_min_cost=1,
        degrade_use_l2=True,
    )

    assert int(cost_mask[20, 20]) == 100
    assert int(np.max(cost_mask)) == 100
    assert not np.any((cost_mask > 0) & (cost_mask < 100))


def test_scale_mode_yaml_and_cost_to_image_mapping():
    yaml_data = build_scale_mask_yaml_data(
        image_ref="keepout_mask.pgm",
        resolution=0.1,
        origin_x=-150.0,
        origin_y=-150.0,
    )
    assert yaml_data["mode"] == "scale"
    assert float(yaml_data["occupied_thresh"]) == 1.0
    assert float(yaml_data["free_thresh"]) == 0.0
    assert int(yaml_data["negate"]) == 0

    cost_mask = np.array([[0, 50, 100]], dtype=np.uint8)
    image = cost_mask_to_scale_image(cost_mask)
    assert int(image[0, 0]) == 255
    assert int(image[0, 1]) == 128
    assert int(image[0, 2]) == 0
