class TrafficSignController:
    def __init__(self, allowed_keys):
        self._allowed_keys = frozenset(allowed_keys)
        self._active = None

    def set_active(self, key):
        if key not in self._allowed_keys:
            raise KeyError(f"Key '{key}' is not allowed.")
        self._active = key

    def get_active(self):
        return self._active

    def clear(self):
        self._active = None

    def as_dict(self):
        return {key: (key == self._active) for key in self._allowed_keys}

    def __contains__(self, key):
        return key in self._allowed_keys

    def __iter__(self):
        return iter(self._allowed_keys)

    def __repr__(self):
        return repr(self.as_dict())