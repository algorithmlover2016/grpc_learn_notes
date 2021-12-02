# [submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules)
## add submodule and check
* ***`git submodule add https_repos`***<br>
    * **`git submodule add https://github.com/chaconinc/DbConnector`**
* **`git status`** to notice the ***`.gitmodules`***
    ```
    [submodule "DbConnector"]
	path = DbConnector
	url = https://github.com/chaconinc/DbConnector

    ```
    * **like your .gitignore file. It’s pushed and pulled with the rest of your project. This is how other people who clone this project know where to get the submodule projects from.**<br>
    * **Since the URL in the .gitmodules file is what other people will first try to clone/fetch from, make sure to use a URL that they can access if possible.**<br>
        * **You can overwrite this value locally with git config submodule.DbConnector.url PRIVATE_URL for your own use. When applicable, a relative URL can be helpful.**<br>
* ***`git diff --cached submodule_Folder`***<br>
    * ***`git diff --cached --submodule`***<br>

## clone a project with submodules
* **`git clone project_repo`**<br>
    * **`cd repo_local_folder`**<br>
    * **`cd subModuleFolder`**<br>
    * **`git submodule update --init`**<br>
        * Or do the two following step respectively<br>
            * **`git submodule init` and then `git submodule update`**<br>
        * **To initialize, fetch and checkout any nested submodules**<br>
            * ***`git submodule update --init --recursive`***<br>
* **`git clone --recurse-submodules project_repo`**<br>

## working on a Project with submodules
### Pulling in Upstream Changes from the Submodule Remote
#### manually fetch and merge in subdirectory
* **`cd subModuleFolder`**<br>
* **`git fetch`**<br>
* **`git merge [origin/master]`**<br>
* **`cd ../; git diff --submodule`**<br>
    * **To avoid typing `--submodule` every time when run git diff, which means only input `git diff`**<br>
        * **`git config --global diff.submodule log`**<br>

#### automatically fetch and merge submodule
* **`cd working_project_folder`**
* **`git submodule update --remote` to update all of your submodules**<br>
    * **pass the name of just the submodule you want to try to update**<br>
        * **`git submodule update --remote subModuleFolderName`**<br>
    * **set specified branch you desired**<br>
        * **set in the `.gitmodules` file**<br>
            * **`git config -f .gitmodules submodule.subModuleFolderName.branch branch_name_you_specify`**<br>
            * **`git submodule update --remote`**<br>
        ***Caution:***<br>
        ***`If you leave off the -f .gitmodules it will only make the change for you`***<br>
* **`git status`**<br>
    * **To show a short summary of changes to your submoudles**<br>
        * **`git config status.submodulesummary 1`**

* **actually see the log of commits that we’re about to commit to in our submodule**<br>
    * **`git log -p --submodule`**<br>
### Pulling Upstream Changes from the Project Remote
* ***By default, the git pull command recursively fetches submodules changes. However, it does not update the submodules***<br>
* **`git pull`**<br>
    * **`git submodule update --init --recursive`**<br>
    * **`git config --global submodule.recurse true`**<br>
* **`git pull --recurse-submodules`**<br>

### Working on a Submodule
* **go into our submodule directory and check out a branch**<br>
    * **`cd subModuleFolderName`**<br>
    * **`git checkout branch_name`**<br>
    * **`cd ../`**<br>
    * **`git submodule update --remote --merge`**<br>
    * **`cd subModuleFolderName` and make your changes and then `git commit -am "commments"`**<br>
    * **`cd ../`**<br>
    * **`git submodule update --remote --rebase`**<br>

### Publishing Submodule Changes
* **`git push --recurse-submodules=check` to make push simply fail if any of the committed submodule changes haven’t been pushed**<br>
    * **`git config push.recurseSubmodules check` to want the check behavior to happen for all pushes**<br>
* **`git push --recurse-submodules=on-demand`**<br>
    * **`git config push.recurseSubmodules on-demand`**<br>
### Merging Submodule Changes
* **`git diff`**<br>
* **`cd subModuleFoloderName`**<br>
* **`git rev-parse HEAD`**<br>
* **`git branch try-merge commit_id`**<br>
* **`git merge try-merge`**<br>

## Submodule Tips
### **Submodule Foreach**<br>
    * **`git submodule foreach 'git stash'`**<br>
    * **`git submodule foreach 'git checkout -b featureA'`**<br>
    * **`git diff; git submodule foreach 'git diff'`**<br>
### **Useful Aliases**<br>
    * **`git config alias.sdiff '!'"git diff && git submodule foreach 'git diff'"`**<br>
    * **`git config alias.spush 'push --recurse-submodules=on-demand'`**<br>
    * **`git config alias.supdate 'submodule update --remote --merge'`**<br>

