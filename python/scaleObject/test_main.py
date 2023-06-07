from scale_object import ScaledObject
import unittest

class TestScaledObject(unittest.TestCase):
    def setUp(self):
        self.obj = ScaledObject()

    def test_min_replica_count(self):
        self.assertEqual(self.obj.min_replica_count, 1)
        self.obj.min_replica_count = 5
        # print(self.obj.get_replica_count(min=True))
        self.assertEqual(self.obj.min_replica_count, 5)
        with self.assertRaises(ValueError):
            self.obj.min_replica_count = 11

    def test_max_replica_count(self):
        self.assertEqual(self.obj.max_replica_count, 10)
        self.obj.max_replica_count = 7
        # print(self.obj.get_replica_count(max=True))
        self.assertEqual(self.obj.max_replica_count, 7)
        with self.assertRaises(ValueError):
            self.obj.max_replica_count = 0

    def test_target_name(self):
        self.assertEqual(self.obj.target_name, "None-target")
        self.obj.env = "prod"
        self.assertEqual(self.obj.target_name, "prod-target")

    def test_get_replica_count(self):
        self.obj.min_replica_count = 5
        self.obj.max_replica_count = 7
        self.assertEqual(self.obj.get_replica_count(), 6)
        self.assertEqual(self.obj.get_replica_count(min=True), 5)
        self.assertEqual(self.obj.get_replica_count(max=True), 7)

if __name__ == '__main__':
    unittest.main()
# python test_main.py
# python -m unittest test_main.py

import pytest

@pytest.fixture
def obj():
    return ScaledObject()

def test_min_replica_count(obj):
    assert obj.min_replica_count == 1
    obj.min_replica_count = 5
    assert obj.min_replica_count == 5
    with pytest.raises(ValueError):
        obj.min_replica_count = 11

def test_max_replica_count(obj):
    assert obj.max_replica_count == 10
    obj.max_replica_count = 7
    assert obj.max_replica_count == 7
    with pytest.raises(ValueError):
        obj.max_replica_count = 0

def test_target_name(obj):
    assert obj.target_name == "None-target"
    obj.env = "prod"
    assert obj.target_name == "prod-target"

def test_get_replica_count(obj):
    obj.max_replica_count = 7
    assert obj.get_replica_count() == 4
    assert obj.get_replica_count(min=True) == 1
    assert obj.get_replica_count(max=True) == 7

# pytest test_main.py