# KUKA 컨트롤러 프록시 서버
cfg_host = {"HOST": "10.0.0.3", "PORT": 9000}

from collections import namedtuple

Config_host = namedtuple("Config_host", cfg_host.keys())(*cfg_host.values())
