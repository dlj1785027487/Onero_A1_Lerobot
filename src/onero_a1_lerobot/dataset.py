"""Lazy imports for optional LeRobot dataset support."""


def _import_lerobot_dataset_tools():
    try:
        from lerobot.datasets.feature_utils import build_dataset_frame, hw_to_dataset_features
    except Exception:  # pragma: no cover - version compatibility
        try:
            from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features
        except Exception as exc:
            raise RuntimeError("LeRobot dataset utilities are not available. Install LeRobot first.") from exc

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except Exception as exc:
        detail = f"{type(exc).__name__}: {exc}"
        raise RuntimeError(
            "LeRobotDataset import failed. LeRobot may be installed, but one of its "
            f"dataset dependencies could not be loaded ({detail})."
        ) from exc

    return LeRobotDataset, build_dataset_frame, hw_to_dataset_features
