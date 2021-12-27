# [git stash](https://git-scm.com/docs/git-stash)
## **SYNOPISIS**<br>
```
git stash list [<log-options>]
git stash show [-u|--include-untracked|--only-untracked] [<diff-options>] [<stash>]
git stash drop [-q|--quiet] [<stash>]
git stash ( pop | apply ) [--index] [-q|--quiet] [<stash>]
git stash branch <branchname> [<stash>]
git stash [push [-p|--patch] [-k|--[no-]keep-index] [-q|--quiet]
	     [-u|--include-untracked] [-a|--all] [-m|--message <message>]
	     [--pathspec-from-file=<file> [--pathspec-file-nul]]
	     [--] [<pathspec>…​]]
git stash clear
git stash create [<message>]
git stash store [-m|--message <message>] [-q|--quiet] <commit>
```
## **git stash**<br>
* **Record the current state of the working directory and the index, but want to go back to a clean working directory**<br>
* **Saves your local modifications away and reverts the working directory to match the `HEAD` commit.**<br>
* **equivalent to `git stash push`**<br>
## **git stash list**<br>
* **List the modifications stashed away by `git stash`**<br>
## **git stash show**<br>
* **Inspect the modifications**<br>
## **git stash apply**<br>
* **Restor (potentially on top of a different commit) modifications**<br>


