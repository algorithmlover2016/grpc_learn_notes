
# [format string float format](https://stackoverflow.com/questions/45310254/fixed-digits-after-decimal-with-f-strings)
```py
#-*- coding:utf-8 -*-
for i in range(1, 25 - 10, 1):
    print(f"{10 + i}: {(1010 * 0.85 - i * 10)/ 1010:.2f}")
```
