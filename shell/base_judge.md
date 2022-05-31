# [file exist or not](https://www.cnblogs.com/emanlee/p/3583769.html)
```sh
# folder exists or not
if [ ! -d "${myfolder}" ]; then
  mkdir "${myfolder}"
fi

# file exists or not
if [ ! -f "${myfile}" ]; then
    touch "${myfile}"
fi

# file or folder is executable
if [ ! -x "$folder"]; then
  mkdir "$folder"
fi

# var is empty or not
if [ ! -n "$var" ]; then
  echo "$var is empty"
fi

# var equal or not
if [ "$var1" = "$var2" ]; then
  echo '$var1 eq $var2'
else
  echo '$var1 not eq $var2'
fi

# there is nothing in the folder, https://blog.csdn.net/wenjjing2lianee/article/details/5633251
if [ "`ls -A $DIRECTORY`" = "" ]; then
  echo "$DIRECTORY is indeed empty"
else
  echo "$DIRECTORY is not empty"
fi

count=`ls $*|wc -w`
if [ "$count" > "0" ];
then
 echo "file size $count"
else
 echo "empty!"
fi
```
* [common compare character](https://blog.csdn.net/u012206617/article/details/118366597)
* [bash diff sh](https://blog.csdn.net/lucykingljj/article/details/48519069)