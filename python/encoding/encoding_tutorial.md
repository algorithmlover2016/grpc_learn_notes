# **set to utf-8**<br>
## get the encoding format for stdout**<br>
```
import sys
print(sys.stdout.encoding)
```
## **set stdcout to encoding with utf-8**<br>
```
import sys
import codecs
sys.stdout = codecs.getwriter('utf8')(sys.stdout)
```
```
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
```
## **print with utf-8**<br>
```
lineStr = lineStr.encode('utf-8')
sys.stdout.write(lineStr)
```
```py
# 1 means stdout
with open(1, 'w', encoding="utf-8") as fw:
    print(lineStr, file = fw)

with open(filePathName, 'w', encoding="utf-8") as fw:
    print(lineStr, file = fw)
```

