# [Using virtualenv](https://www.dabapps.com/blog/introduction-to-pip-and-virtualenv-python/)
* **`easy_install pip`** to install pip globally<br>
## install virutalenv
* **`pip install virtualenv`** to install it globally<br>
## create a new virutal environment
* **changing directory into the root of your project directory**<br>
    ***`cd ~/code/myproject/`***
* **use the virtualenv command-line tool to create a new environment**<br>
    ***`virtualenv env`***<br>
    * **Notes:**<br>
        * **`env` is just the name of the directory you want to create your virtual environment inside**<br>
            * **Eg, your project code is at `~/code/projectname/`**<br>
            * **After run the command of `virutalenv env`, you will get a folder whose path is `~/code/projectname/env/`**<br>
        * **`env` is yourself defined variable and will be projected to subfolder in your project workRoot**<br>
        * **If you're using a version control system like git, you shouldn't commit the env directory. Add it to your .gitignore file (or similar).**<br>
* **show the path tree**<br>
* **use new virtual environment**<br>
    * **use by `evn/bin`**<br>
        * ***`env/bin/pip install -r requirements.txt`*** to install Requirements files<br>
        * ***`env/bin/pip install requests`***<br>
        * ***`env/bin/python python_script_file.py`***<br>
    * Do ***`source env/bin/activate`*** first, and then just run<br>
        * ***`pip install requests`***<br>
        * ***`python python_script_file.py`***<br>
        * **Notes:**<br>
            * **Switch to work on a different project (with its own environment) you can run `deactivate` to stop using one environment**<br>
            * **And then `source env/bin/activate` to activate the other.**<br>
