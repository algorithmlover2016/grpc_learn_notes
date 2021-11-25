# https://zhuanlan.zhihu.com/p/139783331
from pathlib import Path

curPath = Path(__file__).parent / "output/next" / "output.txt"
print(curPath.resolve())
print(curPath.absolute())
print(curPath.is_file())
print(curPath.is_dir())
print(curPath.suffix)

if curPath.parent.parent.exists():
    import shutil
    shutil.rmtree(curPath.parent.parent, ignore_errors=True)

if not curPath.suffix:
    curPath.mkdir(parents=True, exist_ok=True)
    curPath /= "output.txt"
    # curPath.touch(exist_ok=True)

print(curPath.exists())
print(list(curPath.parents))
print(curPath.anchor)
print(curPath.as_posix())
print(curPath.as_uri())
if not curPath.parent.exists():
    curPath.parent.mkdir(parents=True, exist_ok=True)
    curPath.touch(exist_ok=True)

b = {"asfds" : 5, "fdasfa": 10}
b = sorted(b.items(), key = lambda kv: (kv[1], kv[0]), reverse=True)
with open(curPath, 'w') as fw:
    fw.write("hello world\n")
    print(b, file = fw)

leetcodeCppFiles = Path(__file__).parents[3].joinpath("leet_code/src/main/algorithms/cpp")
print(list(leetcodeCppFiles.glob("*.sh")))
print(list(leetcodeCppFiles.rglob("*.txt")))
for subTxtPath in leetcodeCppFiles.rglob("*.sh"):
    print(subTxtPath.absolute())
    print(subTxtPath.resolve())
print("\n")
import os
print(list(entry for entry in os.scandir(leetcodeCppFiles) if entry.is_file()))
print(list(entry for entry in leetcodeCppFiles.iterdir() if entry.is_file()))
print(list(entry for entry in leetcodeCppFiles.iterdir() if entry.is_dir()))

print(Path.cwd(), Path.home())