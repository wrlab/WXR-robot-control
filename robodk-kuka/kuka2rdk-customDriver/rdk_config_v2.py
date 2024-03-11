# WXR 서버
cfg = {"HOST": "192.168.1.169", "PORT": 6894}
# 프록시 서버
cfg_host = {"HOST": "192.168.1.169", "PORT": 9000}
cfg_host2 = {"HOST": "192.168.1.169", "PORT": 9001}

from collections import namedtuple

Config_server = namedtuple("Config", cfg.keys())(*cfg.values())
Config_host = namedtuple("Config_host", cfg_host.keys())(*cfg_host.values())
Config_host2 = namedtuple("Config_host", cfg_host2.keys())(*cfg_host2.values())
