# [Tagging](https://git-scm.com/book/en/v2/Git-Basics-Tagging)
## List tags
* **list all**<br>
```sh
git tag
git ls-remote --tags origin
```
* **search for tags**<br>
`git tag -l "filter_pattern"`<br>

## Create tags

* **create new annotated tags**<br>
`git tag -a 'new_tag_name' -m "comments about this new tags"`

* **create new lightweight tags**<br>
```sh
# To create a lightweight tag, don’t supply any of the -a, -s, or -m options, just provide a tag name
git tag 'new_tag_name'
```

* **create new annotated tag with special commitId**<br>
`git tag -a 'new_tag_name' commit-id`

## **Show the tag**<br>
`git show new_tag_name`

## **[Pull all tags](https://www.shuzhiduo.com/A/QV5Z8aKezy/)**
```sh
git fetch origin remote_tag_name
git fetch origin --prune # delete the local branchs that does not belong to any remote branch
git fetch origin -p
git remote prune origin

```
## Push tags
* **By default, the git push command doesn’t transfer tags to remote servers. You will have to explicitly push tags to a shared server after you have created them.**<br>
* **push the special tag**<br>
`git push origin new_tag_name`
* **push all tags**<br>
```sh
# git push remote_repo_name local_branch_name:remote_target_branch_name --tags
git push origin --tags
```

## Del tags
* **Del local tag**<br>
`git tag -d need_delete_tag_name`
* **Deleting a tag from a remote server**<br>
```sh
# push a null value which is located in between origin and colon to the remote tag name
git push origin : refs/tags/need_delete_remote_tag_name

git push origin --delete need_delete_remote_tag_name
```

## Checkout Tags
* **view the versions of files a tag is pointing to**<br>
```sh
git checkout target_tag_name
# if we have made some changes and want to retain commits
git switch -c new_branch_name
# undo this operation by
git switch -

# accroding to a tag create a branch
git checkout -b new_branch_name target_remote_tag

# same to the following steps(https://cslqm.github.io/2019/08/01/git-tag-manage/):
git fetch origin tag remote_tag_name # pull remote tag named remote_tag_name and save it to local tag with the same name of remote_tag_name
git branch new_branch_name remote_tag_name
```
