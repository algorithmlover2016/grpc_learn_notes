# [cherry-pick](https://git-scm.com/docs/git-cherry-pick)
## [How to use **`cherry-pick`**](https://mattstauffer.com/blog/how-to-merge-only-specific-commits-from-a-pull-request/)<br>
* Pull down the branch locally. Use your git GUI or pull it down on the command line, whatever you'd like.<br>
    ```sh
    git checkout original_brach # git checkout -b orignal_bracnh origin/branch
    git log
    ```
* Get back into the branch you're **merging into**. You'll likely do this by running `git checkout master`.<br>
* Find the commits you want to pull into your branch. Go to either the git log or the GitHub UI and grab the unique commit hashes for each of the commits that you want.<br>
* "Cherry pick" the commits you want into this branch. Run this command: `git cherry-pick super-long-hash-here`. That will pull just this commit into your current branch.<br>
    * pick a range of commits<br>
    ```sh
    # refer to https://stackoverflow.com/questions/1994463/how-to-cherry-pick-a-range-of-commits-and-merge-them-into-another-branch
    git cherry-pick A..B # A should be older than B, and A is excluded and B is included
    git cherry-pick A^..B # A should be older than B, and both A and B are included
    ```
* Push up this branch like normal. `git push origin master`<br>