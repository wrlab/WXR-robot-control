cfg = {"HOST": "localhost", "PORT": 4070}

from collections import namedtuple

Config = namedtuple("Config", cfg.keys())(*cfg.values())
