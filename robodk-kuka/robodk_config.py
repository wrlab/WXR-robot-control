cfg = {"HOST": "localhost", "PORT": 6894}

from collections import namedtuple

Config = namedtuple("Config", cfg.keys())(*cfg.values())
