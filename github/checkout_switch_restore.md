# **git checkout**
* **git checkout -b branchName origin/remote_branch_name**<br>
* **git checkout filename**<br>
* **git checkout branchName**<br>
* **git checkout specific files from other branch**<br>
    * First checkout to the branch you're merging into<br>
    `git checkout merging_into_branch_name`
    * To update the working tree with files or directories from another branch, you can use the branch name pointer<br>
    `git checkout <branch_name> -- <paths>`
    * Update the working tree with files from a tree-ish<br>
    `git checkout [-p|--patch] [<tree-ish>] [--] <pathspec>`
    * **Caution:**<br>
        * When `paths` or `--patch` are given, git checkout does not switch branches.<br>
        * It updates the named paths in the working tree from the index file or from a named <tree-ish> (most often a commit)…<br>

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