# [json dumps and dump](https://www.w3cschool.cn/article/30808038.html)
```py
import json
persons = [
    {
        'username': "zhaoji",
        "age": "18",
        "country": "China"
    },
    {
        "username": "cyj",
        "age": "18",
        "country": "China"
    }
]
x = {'name':'你猜','age':19,'city':'四川'}
# dumps: (obj: Any, *, skipkeys: bool = ..., ensure_ascii: bool = ..., check_circular: bool = ..., allow_nan: bool = ..., cls: Type[JSONEncoder] | None = ..., indent: int | str | None = ..., separators: tuple[str, str] | None = ..., default: (Any) -> Any | None = ..., sort_keys: bool = ..., **kwds: Any) -> str
print(json.dumps(x, separators=(',',':'), ensure_ascii = False))
print(json.dumps(persons, separators=(',',':')))

# dump: (obj: Any, fp: IO[str], *, skipkeys: bool = ..., ensure_ascii: bool = ..., check_circular: bool = ..., allow_nan: bool = ..., cls: Type[JSONEncoder] | None = ..., indent: int | str | None = ..., separators: tuple[str, str] | None = ..., default: (Any) -> Any | None = ..., sort_keys: bool = ..., **kwds: Any) -> None
with open("./write_dump.json", "w", encoding="utf-8") as fp:
    json.dump(x, fp, separators=(',',':'), ensure_ascii = False)
    fp.write("\n")
    json.dump(persons, fp)
```
