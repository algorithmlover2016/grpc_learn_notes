# [git branch]()
## **List Branch**<br>
```sh
git branch -a # list all the branches in local or remote
git branch -r # just list all the remote branches
git branch -vv # list mapping relationship
git branch # list local branch
```
## **[Push branch](https://codeantenna.com/a/7ad7mj49CF)**<br>
```sh
git push --set-upstream origin local_branch_name:remote_target_branch_name
git remote --set-upstream origin local_branch_name:remote_target_branch_name
```
## **Del branch**<br>
```sh
git branch -d local_branch # softly delete local branch
git branch -D local_branch # force deleting local branch
git branch -d -r origin/branch_name # delete remote branch
```

## **[abandon local changes in branch](https://www.cxyzjd.com/article/mlz_2/95316775)**<br>
```sh
git reset --hard [orgin/remote_branch_name]
git branch -u origin/remote_branch_name
```

## **[git create a new branch in local and push to remote](https://blog.csdn.net/huangjw_806/article/details/78297851)
```sh
git checkout -b new_branch_name
# create new_bracnh_name in remote
git push origin new_branch_name
# bind the local branch name into remote branch
git branch --set-upstream-to=origin/new_branch_name new_branch_name
```