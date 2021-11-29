# [git diff](https://git-scm.com/docs/git-diff)
* **`git diff branchA branchB`**<br>
* **[`git diff branchA branchB --stat`](https://blog.csdn.net/yzpbright/article/details/54143129)**<br>
* **`git diff branchA branchB -- file_path`**<br>

## [git log diff](https://www.cnblogs.com/zndxall/p/13897859.html)
* ***exists in branch1 not in branch2***<br>
   * **`git log branch1 ^branch2`**
* ***exists in branch2 not in branch1***<br>
   * **`git log ^branch1 branch2`**
* ***branch2 - branch1***<br>
    * **`git log branch1..branch2`**
* ***branch2 diff with branch1***<br>
    * **`git log branch1...branch2`**
    * **`git log --left-right branch1...branch2`**

# [git rebase](https://git-scm.com/docs/git-rebase)
* **[`git rebase -i [start_commit_id] [end_commit_id]`](https://www.jianshu.com/p/4a8f4af4e803)**
