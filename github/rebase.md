# [rebase command]()
## basic operation<br>
* ***`git rebase -i commit_id`***<br>
* ***pick, drop***<br>
* ***`git rebase --abort`***<br>
* ***`git add conflict_files`***<br>
* ***`git rebase --continue`***<br>
## [operation]()
```sh
# first checkout to target branch
git checkout master
# rebase the commits into target branch
git rebase develop_feature_branch
# git add path_of_conflicting_file
git rebase --continue
git push
```