"""Test the SQLLite cache."""
import os
from dataclasses import (
    dataclass,
    field,
)
from tempfile import NamedTemporaryFile
from unittest import TestCase

from onshape_to_robot.onshape_api.onshape_cache import OnshapeCache


@dataclass
class FakeBytesResult:
    content: bytes = field(default_factory=lambda: os.urandom(1000))


class FakeStringResult:
    content: str = "mystr"


class TestCache(TestCase):
    def setUp(self):
        dbfile = NamedTemporaryFile(delete=False)
        dbfile.close()
        self._dbfile = dbfile.name
        self.cache = OnshapeCache(self._dbfile)

    def tearDown(self):
        os.unlink(self._dbfile)

    def test_get_or_add_bytes(self):
        key, method, data = "mykey", "mymethod", FakeBytesResult()
        result = self.cache.get_or_add(key, method, lambda: data)
        self.assertEqual(result, data.content)
        result = self.cache.get_or_add(key, method, None)
        self.assertEqual(result, data.content)

        key, method, newdata = "myotherkey", "myothermethod", FakeBytesResult()
        self.assertNotEqual(data, newdata)
        self.cache.get_or_add(key, method, lambda: newdata)
        self.assertEqual(result, data.content)

    def test_get_or_add_str(self):
        key, method, data = "mykey", "mymethod", FakeStringResult()
        result = self.cache.get_or_add(key, method, lambda: data, is_string=True)
        self.assertEqual(result, data.content)
        result = self.cache.get_or_add(key, method, None, is_string=True)
        self.assertEqual(result, data.content)
