# -*- coding:utf-8 -*-

import logging
import pandas as pd
from pathlib import Path

filePath = Path(__file__).parent / "data/categories_places365_huayang.txt"

if not filePath.parent.exists():
    filePath.parent.mkdir(parents=True, exist_ok=True)
if not filePath.exists():
    logging.warning(f"{filePath} does not exist")
    with open(filePath, 'w', encoding="utf-8") as fw:
        fw.write("")

csvInData = pd.read_csv(filePath, delimiter="\t", index_col='level2_cate_id', delim_whitespace=True)
print(csvInData)