"""A1 adapter exports, loaded on demand to keep CAN workers independent of LeRobot."""

from importlib import import_module

_EXPORT_MODULES = {
    "LEROBOT_AVAILABLE": ".compat",
    "ArmMappingConfig": ".sdk_device_config",
    "SdkArmConfig": ".sdk_device_config",
    "UsbCameraConfig": ".camera_config",
    "OneroSdk": ".sdk_robot",
    "OneroSdkRobot": ".sdk_robot",
    "OneroSdkConfig": ".sdk_config",
    "OneroSdkTeleop": ".sdk_teleoperator",
    "OneroSdkTeleopConfig": ".sdk_config",
}

__all__ = list(_EXPORT_MODULES)


def __getattr__(name: str):
    module_name = _EXPORT_MODULES.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(module_name, __name__), name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
