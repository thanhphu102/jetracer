import os

import yaml


def default_config_path():
    package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    return os.path.join(package_root, "config", "params.yaml")


class Config:
    """Small dotted-key wrapper around the project YAML file."""

    def __init__(self, data, path=None):
        self.data = data or {}
        self.path = path
        if path:
            self.package_root = os.path.abspath(os.path.join(os.path.dirname(path), ".."))
        else:
            self.package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

    @classmethod
    def load(cls, path=None):
        config_path = path or default_config_path()
        with open(config_path, "r") as config_file:
            data = yaml.safe_load(config_file) or {}
        return cls(data, config_path)

    def get(self, dotted_key, default=None):
        value = self.data
        for part in dotted_key.split("."):
            if not isinstance(value, dict) or part not in value:
                return default
            value = value[part]
        return value

    def resolve_path(self, path_value):
        if not path_value:
            return path_value
        if os.path.isabs(path_value):
            return path_value
        return os.path.abspath(os.path.join(self.package_root, path_value))
