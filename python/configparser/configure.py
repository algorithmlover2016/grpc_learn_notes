#-*- coding:utf-8 -*-

import configparser
# from typing import OrderedDict
import collections
config = configparser.ConfigParser()
config['DEFAULT'] = {'ServerAliveInterval': '45',
                     'Compression': 'yes',
                     'CompressionLevel': '9'}
config['bitbucket.org'] = {}
config['bitbucket.org']['User'] = 'hg'
config['topsecret.server.com'] = {}
topsecret = config['topsecret.server.com']
topsecret['Port'] = '50022'     # mutates the parser
topsecret['ForwardX11'] = 'no'  # same here
config['DEFAULT']['ForwardX11'] = 'yes'
with open('example.ini', 'w') as configfile:
  config.write(configfile)

del config
config = configparser.ConfigParser()
print(config.sections())

config.read('example.ini')

print(config.sections())
print('bitbucket.org' in config)

print('bytebong.com' in config)

# keys in sections are case-insensitive and stored in lowercase
print(config['bitbucket.org']['User'])

print(config['DEFAULT']['Compression'])

topsecret = config['topsecret.server.com']
print(topsecret['ForwardX11'])

print(topsecret['Port'])

for key in config['bitbucket.org']:  
    print(key)

print(config['bitbucket.org']['ForwardX11'])

another_config = configparser.ConfigParser()
another_config.read('example.ini')

print(another_config['topsecret.server.com']['Port'])

another_config.read_string("[topsecret.server.com]\nPort=48484")
print(another_config['topsecret.server.com']['Port'])

another_config.read_dict({"topsecret.server.com": {"Port": 21212}})
print(another_config['topsecret.server.com']['Port'])

print(another_config['topsecret.server.com']['ForwardX11'])

print(topsecret.getboolean('ForwardX11'))

import sys
config.write(sys.stdout)

print(config['bitbucket.org'].getboolean('ForwardX11'))
print(config['bitbucket.org']['user'])
print(config['bitbucket.org'].get('ForwardX11'))

print(config.getboolean('bitbucket.org', 'Compression'))
print(config.get('bitbucket.org', 'Compression'))
print(config.get('bitbucket.org', 'monster', \
           fallback='No such things as monsters'))

print(topsecret.getboolean('BatchMode', fallback=True))

config['DEFAULT']['BatchMode'] = 'no'
print(topsecret.getboolean('BatchMode', fallback=True))
print(config.BOOLEAN_STATES)

config = """
[Section1]
Key = Value

[Section2]
AnotherKey = Value
"""
typical = configparser.ConfigParser()
typical.read_string(config)
print(list(typical['Section1'].keys()))

print(list(typical['Section2'].keys()))

custom = configparser.RawConfigParser()
custom.optionxform = lambda option: option
custom.read_string(config)
print(list(custom['Section1'].keys()))

print(list(custom['Section2'].keys()))

import re
config = """
[Section 1]
option = value

[  Section 2  ]
another = val
"""
typical = configparser.ConfigParser()
typical.read_string(config)
print(typical.sections())

custom = configparser.ConfigParser()
custom.SECTCRE = re.compile(r"\[ *(?P<header>[^]]+?) *\]")
custom.read_string(config)
print(custom.sections())

del config
config = configparser.RawConfigParser()

# Please note that using RawConfigParser's set functions, you can assign
# non-string values to keys internally, but will receive an error when
# attempting to write to a file or when you get it in non-raw mode. Setting
# values using the mapping protocol or ConfigParser's set() does not allow
# such assignments to take place.
config.add_section('Section1')
config.set('Section1', 'an_int', '15')
config.set('Section1', 'a_bool', 'true')
config.set('Section1', 'a_float', '3.1415')
config.set('Section1', 'baz', 'fun')
config.set('Section1', 'bar', 'Python')
config.set('Section1', 'foo', '%(bar)s is %(baz)s!')
# Writing our configuration file to 'example.cfg'
with open('example.cfg', 'w') as configfile:
    config.write(configfile)

del config
config = configparser.RawConfigParser()
config.read('example.cfg')

# getfloat() raises an exception if the value is not a float
# getint() and getboolean() also do this for their respective types
a_float = config.getfloat('Section1', 'a_float')
an_int = config.getint('Section1', 'an_int')
print(a_float + an_int)

# Notice that the next output does not interpolate '%(bar)s' or '%(baz)s'.
# This is because we are using a RawConfigParser().
if config.getboolean('Section1', 'a_bool'):
    print(config.get('Section1', 'foo'))

# Writing our configuration file to 'example.cfg'
with open('example.cfg', 'w') as configfile:
    config.write(configfile)

cfg = configparser.ConfigParser()
cfg.read('example.cfg')

# Set the optional *raw* argument of get() to True if you wish to disable
# interpolation in a single get operation.
print(cfg.get('Section1', 'foo', raw=False))  # -> "Python is fun!"
print(cfg.get('Section1', 'foo', raw=True))   # -> "%(bar)s is %(baz)s!"

# The optional *vars* argument is a dict with members that will take
# precedence in interpolation.
print(cfg.get('Section1', 'foo', vars={'bar': 'Documentation',
                                       'baz': 'evil'}))

# The optional *fallback* argument can be used to provide a fallback value
print(cfg.get('Section1', 'foo'))
      # -> "Python is fun!"

print(cfg.get('Section1', 'foo', fallback='Monty is not.'))
      # -> "Python is fun!"

print(cfg.get('Section1', 'monster', fallback='No such things as monsters.'))
      # -> "No such things as monsters."

# A bare print(cfg.get('Section1', 'monster')) would raise NoOptionError
# but we can also use:

print(cfg.get('Section1', 'monster', fallback=None))
      # -> None

del config
# New instance with 'bar' and 'baz' defaulting to 'Life' and 'hard' each
config = configparser.ConfigParser({'bar': 'Life', 'baz': 'hard'})
config.read('example.cfg')

print(config.get('Section1', 'foo'))     # -> "Python is fun!"
config.remove_option('Section1', 'bar')
config.remove_option('Section1', 'baz')
print(config.get('Section1', 'foo'))     # -> "Life is hard!"
print(list(config.items()))

parser = configparser.ConfigParser(dict_type = collections.OrderedDict)
parser.read_dict({'section5': {'key1': 'value1',
                               'key3': 'value3',
                               'key2': 'value2'},
                  'section3': {'keyA': 'valueA',
                               'keyC': 'valueC',
                               'keyB': 'valueB'},
                  'section2': {'foo': 'x',
                               'bar': 'y',
                               '中国': '发生发撒',
                               'baz': 'z'}
})
print(parser.sections())

print([option for option in parser['section3']])
print([option for option in parser['section2']])
with open('ordered.ini', 'w', encoding="utf-8") as configfile:
  parser.write(configfile)

del parser
parser = configparser.ConfigParser()
parser.read('ordered.ini', encoding="utf-8")
print(parser.sections())
print([option for option in parser['section3']])
print([option for option in parser['section2']])
print(collections.OrderedDict(parser))

import json
import collections

data =  {
        "id" : "de",
        "Key" : "1234567",
        "from" : "test@test.com",
        "expires" : "2018-04-25 18:45:48.3166159",
        "command" : "method.exec",
        "params" : {
          "method" : "cmd",
          "Key" : "default",
          "params" : {
            "command" : "testing 23"
          }
        }}

data_str = json.dumps(data)
print(data_str)

result = json.loads(data_str, object_pairs_hook=collections.OrderedDict)
print(result)
with open("ordered_dict.txt", 'w', encoding="utf-8") as fw:
  print(result, file = fw)
  print(json.dumps(result), file = fw)

with open("ordered_dict.txt", 'r', encoding="utf-8") as fr:
  line = fr.readline(); line = fr.readline()
  print(json.loads(line))