# WXR server info
cfg_wxrServer = {"HOST": "localhost", "PORT": 6894}

from collections import namedtuple

Config_server = namedtuple("Config", cfg_wxrServer.keys())(*cfg_wxrServer.values())
