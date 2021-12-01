# [configuration file parser](https://docs.python.org/3/library/configparser.html)
* **Config parsers do not guess datatypes of values in configuration files, always storing them internally as strings.**<br>
* **keys in sections are case-insensitive and stored in lowercase, if you want change the default strategy, please visit [this](https://docs.python.org/3/library/configparser.html#customizing-parser-behaviour)**<br>
## get sections except from Default
* **`config.sections()`**<br>
* **DEfault means all the sections will have these keys/values pairs**
## get boolean int float
* **[`getboolean`](https://docs.python.org/3/library/configparser.html#configparser.ConfigParser.getboolean)**
* **[`getint`](https://docs.python.org/3/library/configparser.html#configparser.ConfigParser.getint)**
* **[`getfloat`](https://docs.python.org/3/library/configparser.html#configparser.ConfigParser.getfloat)**

