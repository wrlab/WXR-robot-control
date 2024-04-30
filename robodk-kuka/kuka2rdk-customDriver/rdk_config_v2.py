# KUKA 컨트롤러 프록시 서버
#cfg_host = {"HOST": "10.0.0.3", "PORT": 9000}
cfg_host = {"HOST": "172.31.2.147", "PORT": 7000}
cfg_host2 = {"HOST": "10.0.0.4", "PORT": 9000}

from collections import namedtuple

Config_host = namedtuple("Config_host", cfg_host.keys())(*cfg_host.values())
Config_host2 = namedtuple("Config_host", cfg_host2.keys())(*cfg_host2.values())
