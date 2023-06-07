class ScaledObject:
    def __init__(self, env = None, namespace = None):
        self.env = env
        self.namespace = namespace
        self._min_replica_count = 1
        self._max_replica_count = 10

    @property
    def min_replica_count(self):
        return self._min_replica_count

    @min_replica_count.setter
    def min_replica_count(self, value):
        if value > self._max_replica_count:
            raise ValueError("min_replica_count cannot be greater than max_replica_count")
        self._min_replica_count = value

    @property
    def max_replica_count(self):
        return self._max_replica_count

    @max_replica_count.setter
    def max_replica_count(self, value):
        if value < self._min_replica_count:
            raise ValueError("max_replica_count cannot be less than min_replica_count")
        self._max_replica_count = value

    @property
    def target_name(self):
        return f"{self.env}-target"

    def get_replica_count(self, min=False, max=False):
        if min:
            return self.min_replica_count
        elif max:
            return self.max_replica_count
        else:
            return (self.min_replica_count + self.max_replica_count) // 2