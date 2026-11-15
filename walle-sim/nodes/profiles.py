"""Load a hardware profile from config/sim_profiles.yaml: `defaults` merged with one profile."""

import copy
import os

import yaml

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROFILES_FILE = os.path.join(SIM_DIR, "config", "sim_profiles.yaml")


def _merge(base, extra):
    out = copy.deepcopy(base)
    for key, value in (extra or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = value
    return out


def profile_names(path=PROFILES_FILE):
    with open(path) as f:
        return sorted(yaml.safe_load(f)["profiles"])


def load_profile(name, path=PROFILES_FILE):
    with open(path) as f:
        data = yaml.safe_load(f)
    if name not in data["profiles"]:
        raise ValueError(f"Unknown profile '{name}'. Choose one of: "
                         f"{', '.join(sorted(data['profiles']))}")
    profile = _merge(data["defaults"], data["profiles"][name])
    profile["name"] = name
    return profile
