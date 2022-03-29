# [rebase command](https://git-scm.com/book/en/v2/Git-Branching-Rebasing)
## basic operation<br>
* ***`git rebase -i commit_id`***<br>
* ***pick, drop***<br>
* ***`git rebase --abort`***<br>
* ***`git add conflict_files`***<br>
* ***`git rebase --continue`***<br>
## [operation](https://git-scm.com/book/en/v2/Git-Branching-Rebasing)
```sh
# first checkout to needing replaying branch
git checkout develop_feature_branch
# rebase the commits into target branch
git rebase master
# or equivalently
git rebase master develop_feature_branch # replay the commits in develop_feature_branch into master
# git add path_of_conflicting_file
git checkout master
git merge master develop_feature_branch
git rebase --continue
git push

# add branch client - server difference into master
git rebase --onto master server client
git checkout master
git merge client

git rebase master server
git checkout master
git merge server

git branch -d client
git branch -d server
```
