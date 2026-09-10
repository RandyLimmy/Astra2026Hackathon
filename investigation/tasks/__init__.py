"""Operator-selected control tasks; legacy presets are separate."""
from importlib import import_module

TASKS = {
    "quadruped_gait_failure": {"platform": "quadruped", "module": "dog", "label": "Robot dog"},
    "drone_delivery_imbalance": {"platform": "drone", "module": "drone", "label": "Drone delivery"},
    "warehouse_curve_demo": {"platform": "warehouse", "module": "warehouse", "label": "Warehouse trolley"},
    "car_auto_brake_failure": {"platform": "car", "module": "car", "label": "Car braking"},
}

TASK_PLATFORMS = tuple(dict.fromkeys(task["platform"] for task in TASKS.values()))


def adapter_for(scenario):
    if scenario not in TASKS:
        raise ValueError("Choose one of the declared control tasks.")
    return import_module(f"investigation.tasks.{TASKS[scenario]['module']}").Adapter()
