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
* ***``git diff --cached submodule_Folder***<br>
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
* **`git submodule update --remote subModuleFolderName`**<br>
    * **set specified branch you desired**<br>
        * **set in the `.gitmodules` file**<br>
            * **`git config -f .gitmodules submodule.subModuleFolderName.branch branch_name_you_specify`**<br>
            * **`git submodule update --remote`**<br>
        ***Note***<br>
        ***`If you leave off the -f .gitmodules it will only make the change for you`***<br>
        



