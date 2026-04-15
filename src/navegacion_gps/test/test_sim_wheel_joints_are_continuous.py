from pathlib import Path
import xml.etree.ElementTree as ET


MODEL_FILES = [
    Path(__file__).resolve().parents[1] / "models" / "cuatri_real.urdf",
    Path(__file__).resolve().parents[1] / "models" / "cuatri.urdf",
    Path(__file__).resolve().parents[1] / "models" / "cuatri_ultrasound.urdf",
]

WHEEL_JOINT_NAMES = {
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
    "front_left_wheel_joint",
    "front_right_wheel_joint",
}


def test_sim_wheel_joints_are_continuous() -> None:
    for model_file in MODEL_FILES:
        root = ET.fromstring(model_file.read_text(encoding="utf-8"))
        joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

        for joint_name in WHEEL_JOINT_NAMES:
            joint = joints[joint_name]
            assert joint.attrib["type"] == "continuous", (
                f"{model_file.name}:{joint_name} must be continuous "
                "to avoid wheel rotation saturation in long simulations"
            )
