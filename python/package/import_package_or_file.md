# [How python finds its modules](https://askubuntu.com/questions/470982/how-to-add-a-python-module-to-syspath/471168#471168?newreg=fd69f27dee5d461d865f883e25e8ec13)
* ***a module is a single python file.***<br>
* ***a package is a folder containing python files, accompanied by a (can be empty) file named __init__.py, to tell python it is a package to import modules from.***<br>
## To find out what is included in $PYTHONPATH, (Python looks for its modules and packages in $PYTHONPATH)
```
import sys
print(sys.path)
```
## How to add a directory
* **add path(s) occasionally to the default path by insert operation**<br>
```
import sys
sys.path.insert(0, "/path/to/your/package_or_module")
```
* **add a module(`module_1.py`) located in `/home/myname/pythonfiles` folder**<br>
```py
import sys
# the following path of "/home/myname/pythonfiles/" can be replaced by relative path related to __file__
sys.path.insert(0, "/home/myname/pythonfiles/")
import module_1
```
* **add a package(`my_package`) located in `/home/myname/pythonfiles` folder**<br>
    * **import a module named `module_2.py` located in `my_pakcage`**<br>
    * **import a module named `module_3.py` located in `subfolder` inside `my_package`**<br>
    ```py
    # mypackage/module_2.py
    import sys
    # the following path of "/home/myname/pythonfiles/" can be replaced by relative path related to __file__
    sys.path.insert(0, "/home/myname/pythonfiles/")
    from my_package import module_2
    import my_package.subfolder.module_3
    ```
    ***Caution:***<br>
    * **Given the fact that all subfolders in the package include their own `__init__.py` file.**
* **add a module which is in the same directory as the script or application**<br>
```py
import module_4
from .module_4 import className
from .module_4 import functionName
```
## how to write `__init__.py`
```py
# reference to https://github.com/facebookresearch/detectron2/blob/main/detectron2/modeling/backbone/__init__.py

# package/backbone/__init__.py
# Copyright (c) Facebook, Inc. and its affiliates.
from .build import build_backbone, BACKBONE_REGISTRY  # noqa F401 isort:skip

from .backbone import Backbone
from .fpn import FPN
from .regnet import RegNet
from .resnet import (
    BasicStem,
    ResNet,
    ResNetBlockBase,
    build_resnet_backbone,
    make_stage,
    BottleneckBlock,
)

__all__ = [k for k in globals().keys() if not k.startswith("_")]
# TODO can expose more resnet blocks after careful consideration
```

```py
# reference to https://github.com/facebookresearch/detectron2/blob/main/detectron2/modeling/__init__.py

# package/modeling/__init__.py
# Copyright (c) Facebook, Inc. and its affiliates.
from detectron2.layers import ShapeSpec

from .anchor_generator import build_anchor_generator, ANCHOR_GENERATOR_REGISTRY
from .backbone import (
    BACKBONE_REGISTRY,
    FPN,
    Backbone,
    ResNet,
    ResNetBlockBase,
    build_backbone,
    build_resnet_backbone,
    make_stage,
)
from .meta_arch import (
    META_ARCH_REGISTRY,
    SEM_SEG_HEADS_REGISTRY,
    GeneralizedRCNN,
    PanopticFPN,
    ProposalNetwork,
    RetinaNet,
    SemanticSegmentor,
    build_model,
    build_sem_seg_head,
)
from .postprocessing import detector_postprocess
from .proposal_generator import (
    PROPOSAL_GENERATOR_REGISTRY,
    build_proposal_generator,
    RPN_HEAD_REGISTRY,
    build_rpn_head,
)
from .roi_heads import (
    ROI_BOX_HEAD_REGISTRY,
    ROI_HEADS_REGISTRY,
    ROI_KEYPOINT_HEAD_REGISTRY,
    ROI_MASK_HEAD_REGISTRY,
    ROIHeads,
    StandardROIHeads,
    BaseMaskRCNNHead,
    BaseKeypointRCNNHead,
    FastRCNNOutputLayers,
    build_box_head,
    build_keypoint_head,
    build_mask_head,
    build_roi_heads,
)
from .test_time_augmentation import DatasetMapperTTA, GeneralizedRCNNWithTTA
from .mmdet_wrapper import MMDetBackbone, MMDetDetector

_EXCLUDE = {"ShapeSpec"}
__all__ = [k for k in globals().keys() if k not in _EXCLUDE and not k.startswith("_")]


from detectron2.utils.env import fixup_module_metadata

fixup_module_metadata(__name__, globals(), __all__)
del fixup_module_metadata

```