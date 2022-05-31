# [position parameter](https://www.linuxidc.com/Linux/2018-11/155618.htm)
```sh
$#	传递到脚本的参数个数
$*	以一个单字符串显示所有向脚本传递的参数。 如"$*"用「"」括起来的情况、以"$1 $2 … $n"的形式输出所有参数。
$$	脚本运行的当前进程ID号
$!	后台运行的最后一个进程的ID号
$@	与$*相同，但是使用时加引号，并在引号中返回每个参数。 如"$@"用「"」括起来的情况、以"$1" "$2" … "$n" 的形式输出所有参数。
$-	显示Shell使用的当前选项，与set命令功能相同。
$?	显示最后命令的退出状态。0表示没有错误，其他任何值表明有错误。
```
## [default val]
```sh
# var will not be set
# var has not been defined
unset $var
echo ${var-defaultValue}
# var has not been defined or is empty
echo ${var:-defaultValue}

# var will be set.
# var has not been defined
unset $var
echo ${var=defaultValue}
# var has not been defined or is empty
echo ${var:=defaultValue}

# var has not been defined
unset $var
echo ${var?"error message"}
# var has not been defined or is empty
echo ${var:?"error message"}

# var has been defined, return defaultValue
echo ${var:+defaultValue}
# var has been defined or is not empty, return defaultValue
echo ${var:+defaultValue}
```