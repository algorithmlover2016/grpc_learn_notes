# [submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules)
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



