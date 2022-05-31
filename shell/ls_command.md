#[ls current subFolder](https://www.xstnet.com/columns/linux-notes/basic/file-and-dir/only-show-dir.html)
```sh
ls -d */
ls -F $target_folder | grep '/$'
ls -l $target_folder | grep "^d"
find $target_folder -type d -maxdepth 1
```