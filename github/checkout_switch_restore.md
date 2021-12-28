# **git checkout**
* **git checkout -b branchName origin/remote_branch_name**<br>
* **git checkout filename**<br>
* **git checkout branchName**<br>

# **[git switch](https://git-scm.com/docs/git-switch)**
* **Switch to a specified branch**<br>
    * **`git switch branchName`**<br>
* **Optionally a new branch could be created with either `-c, -C`**<br>
    * **`git switch -c branchName`**<br>

# **[git restore](https://git-scm.com/docs/git-restore)**<br>
[//]: # (comments in md, not been seen in preview)
[//]: # "comments in md, not been seen in preview"
```sh
git restore hello.c
git restore '*.c'
git restore .
git restore :/
git restore --staged hello.c # same to git reset

git restore --source=HEAD --staged --worktree hello.c # same to git checkout
# same to
git restore -s@ -SW hello.c
```