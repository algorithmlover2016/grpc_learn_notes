# **Learn the Registry framework from detectron2**<br>
```py
# Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
# pyre-ignore-all-errors[2,3]
from typing import Any, Dict, Iterable, Iterator, Tuple

from tabulate import tabulate


class Registry(Iterable[Tuple[str, Any]]):
    """
    The registry that provides name -> object mapping, to support third-party
    users' custom modules.

    To create a registry (e.g. a backbone registry):

    .. code-block:: python

        BACKBONE_REGISTRY = Registry('BACKBONE')

    To register an object:

    .. code-block:: python

        @BACKBONE_REGISTRY.register()
        class MyBackbone():
            ...

    Or:

    .. code-block:: python

        BACKBONE_REGISTRY.register(MyBackbone)
    """

    def __init__(self, name: str) -> None:
        """
        Args:
            name (str): the name of this registry
        """
        self._name: str = name
        self._obj_map: Dict[str, Any] = {}

    def _do_register(self, name: str, obj: Any) -> None:
        assert (
            name not in self._obj_map
        ), "An object named '{}' was already registered in '{}' registry!".format(
            name, self._name
        )
        self._obj_map[name] = obj

    def register(self, obj: Any = None) -> Any:
        """
        Register the given object under the the name `obj.__name__`.
        Can be used as either a decorator or not. See docstring of this class for usage.
        """
        if obj is None:
            # used as a decorator
            def deco(func_or_class: Any) -> Any:
                name = func_or_class.__name__
                self._do_register(name, func_or_class)
                return func_or_class

            return deco

        # used as a function call
        name = obj.__name__
        self._do_register(name, obj)

    def get(self, name: str) -> Any:
        ret = self._obj_map.get(name)
        if ret is None:
            raise KeyError(
                "No object named '{}' found in '{}' registry!".format(name, self._name)
            )
        return ret

    def __contains__(self, name: str) -> bool:
        return name in self._obj_map

    def __repr__(self) -> str:
        table_headers = ["Names", "Objects"]
        table = tabulate(
            self._obj_map.items(), headers=table_headers, tablefmt="fancy_grid"
        )
        return "Registry of {}:\n".format(self._name) + table

    def __iter__(self) -> Iterator[Tuple[str, Any]]:
        return iter(self._obj_map.items())

    # pyre-fixme[4]: Attribute must be annotated.
    __str__ = __repr__

```

## **How to use Registry**<br>

```Py
# define a Registry
# build.py
from detectron2.utils.registry import Registry

from .backbone import Backbone

BACKBONE_REGISTRY = Registry("BACKBONE")
BACKBONE_REGISTRY.__doc__ = """
Registry for backbones, which extract feature maps from images

The registered object must be a callable that accepts two arguments:

1. A :class:`detectron2.config.CfgNode`
2. A :class:`detectron2.layers.ShapeSpec`, which contains the input shape specification.

Registered object must return instance of :class:`Backbone`.
"""


def build_backbone(cfg, input_shape=None):
    """
    Build a backbone from `cfg.MODEL.BACKBONE.NAME`.

    Returns:
        an instance of :class:`Backbone`
    """
    if input_shape is None:
        input_shape = ShapeSpec(channels=len(cfg.MODEL.PIXEL_MEAN))

    backbone_name = cfg.MODEL.BACKBONE.NAME
    backbone = BACKBONE_REGISTRY.get(backbone_name)(cfg, input_shape)
    assert isinstance(backbone, Backbone)
    return backbone

```
```py
# add content for Registry

# register a function
from .build import BACKBONE_REGISTRY
@BACKBONE_REGISTRY.register()
def build_resnet_backbone(cfg, input_shape):
    """
    your codes here
    """

# register a class
"""
# first define META_ARCH_REGISTRY in a build.py for example
"""
META_ARCH_REGISTRY = Registry("META_ARCH")  # noqa F401 isort:skip
META_ARCH_REGISTRY.__doc__ = """
Registry for meta-architectures, i.e. the whole model.

The registered object will be called with `obj(cfg)`
and expected to return a `nn.Module` object.
"""
@META_ARCH_REGISTRY.register()
class GeneralizedRCNN(nn.Module):
    """
    your codes here
    """
```

## **[tabulate tablefmt](https://python.fasionchan.com/zh_CN/latest/libs/tabulate.html)**<br>
### **中文对齐**<br>
```
pip install wcwidth
import wcwidth
```

```py
from tabulate import tabulate
table_data = [
    ('Tom', '90', '80', '85'),
    ('Jim', '70', '90', '80'),
    ('Lucy', '90', '70', '90'),
]
table_header = ['Name', 'Chinese', 'Math', 'English']
```
* **plain**<br>
```py
print(tabulate(table_data, headers=table_header, tablefmt='plain'))
# output:
Name      Chinese    Math    English
Tom            90      80         85
Jim            70      90         80
Lucy           90      70         90
```

* **simple**<br>
```py
print(tabulate(table_data, headers=table_header, tablefmt='simple'))
# output
Name      Chinese    Math    English
------  ---------  ------  ---------
Tom            90      80         85
Jim            70      90         80
Lucy           90      70         90
```
* **grid**<br>
```py
print(tabulate(table_data, headers=table_header, tablefmt='grid'))
# output
+--------+-----------+--------+-----------+
| Name   |   Chinese |   Math |   English |
+========+===========+========+===========+
| Tom    |        90 |     80 |        85 |
+--------+-----------+--------+-----------+
| Jim    |        70 |     90 |        80 |
+--------+-----------+--------+-----------+
| Lucy   |        90 |     70 |        90 |
+--------+-----------+--------+-----------+
```
* **fancy_grid**<br>
```py
print(tabulate(table_data, headers=table_header, tablefmt='fancy_grid'))
# output
╒════════╤═══════════╤════════╤═══════════╕
│ Name   │   Chinese │   Math │   English │
╞════════╪═══════════╪════════╪═══════════╡
│ Tom    │        90 │     80 │        85 │
├────────┼───────────┼────────┼───────────┤
│ Jim    │        70 │     90 │        80 │
├────────┼───────────┼────────┼───────────┤
│ Lucy   │        90 │     70 │        90 │
╘════════╧═══════════╧════════╧═══════════╛
```
* **pipe**<br>
```py
print(tabulate(table_data, headers=table_header, tablefmt='pipe'))
# output
| Name   |   Chinese |   Math |   English |
|:-------|----------:|-------:|----------:|
| Tom    |        90 |     80 |        85 |
| Jim    |        70 |     90 |        80 |
| Lucy   |        90 |     70 |        90 |
```
* **orgtlb**<br>
```py
print(tabulate(table_data, headers=table_header, tablefmt='orgtlb'))
# output
Name      Chinese    Math    English
------  ---------  ------  ---------
Tom            90      80         85
Jim            70      90         80
Lucy           90      70         90
```
* **jira**<br>
```py
print(tabulate(table_data, headers=table_header, tablefmt='jira'))
# output
|| Name   ||   Chinese ||   Math ||   English ||
| Tom    |        90 |     80 |        85 |
| Jim    |        70 |     90 |        80 |
| Lucy   |        90 |     70 |        90 |
```
* **presto**<br>
```py
print(tabulate(table_data, headers=table_header, tablefmt='presto'))
# output
 Name   |   Chinese |   Math |   English
--------+-----------+--------+-----------
 Tom    |        90 |     80 |        85
 Jim    |        70 |     90 |        80
 Lucy   |        90 |     70 |        90
```
* **psql**<br>
```py
print(tabulate(table_data, headers=table_header, tablefmt='psql'))
# output
+--------+-----------+--------+-----------+
| Name   |   Chinese |   Math |   English |
|--------+-----------+--------+-----------|
| Tom    |        90 |     80 |        85 |
| Jim    |        70 |     90 |        80 |
| Lucy   |        90 |     70 |        90 |
+--------+-----------+--------+-----------+
```
* **rst**<br>
```py
print(tabulate(table_data, headers=table_header, tablefmt='rst'))
# output
======  =========  ======  =========
Name      Chinese    Math    English
======  =========  ======  =========
Tom            90      80         85
Jim            70      90         80
Lucy           90      70         90
======  =========  ======  =========
```
* **html**<br>
```py
print(tabulate(table_data, headers=table_header, tablefmt='html'))
# output
<table>
<thead>
<tr><th>Name  </th><th style="text-align: right;">  Chinese</th><th style="text-align: right;">  Math</th><th style="text-align: right;">  English</th></tr>
</thead>
<tbody>
<tr><td>Tom   </td><td style="text-align: right;">       90</td><td style="text-align: right;">    80</td><td style="text-align: right;">       85</td></tr>
<tr><td>Jim   </td><td style="text-align: right;">       70</td><td style="text-align: right;">    90</td><td style="text-align: right;">       80</td></tr>
<tr><td>Lucy  </td><td style="text-align: right;">       90</td><td style="text-align: right;">    70</td><td style="text-align: right;">       90</td></tr>
</tbody>
</table>
```