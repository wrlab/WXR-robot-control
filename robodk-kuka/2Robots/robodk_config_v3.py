cfg = {"HOST": "localhost", "PORT": 6894}
cfg_host = {"HOST": "192.168.1.169", "PORT": 9000}

from collections import namedtuple

Config_server = namedtuple("Config", cfg.keys())(*cfg.values())
Config_host = namedtuple("Config_host", cfg_host.keys())(*cfg_host.values())
