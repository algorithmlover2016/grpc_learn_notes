# [change \r\n to \n in linux](https://segmentfault.com/q/1010000011799577)
```sh
find . -type f -exec dos2unix {} \;
```
