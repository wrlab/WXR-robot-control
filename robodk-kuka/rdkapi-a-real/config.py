#cfg_wxrServer = {"HOST": "localhost", "PORT": 6894}
#cfg_wxrServer = {"HOST": "192.168.1.123", "PORT": 6894}
cfg_wxrServer = {"HOST": "10.0.0.4", "PORT": 6894}
#cfg_wxrServer = {"HOST": "192.168.0.35", "PORT": 6894}
#cfg_wxrServer = {"HOST": "192.168.146.217", "PORT": 6894}
#cfg_wxrServer = {"HOST": "192.168.1.152", "PORT": 6894}
#cfg_wxrServer = {"HOST": "172.16.0.191", "PORT": 6894} # 2층 리빙랩

from collections import namedtuple

Config_server = namedtuple("Config", cfg_wxrServer.keys())(*cfg_wxrServer.values())
