# [How python finds its modules](https://askubuntu.com/questions/470982/how-to-add-a-python-module-to-syspath/471168#471168?newreg=fd69f27dee5d461d865f883e25e8ec13)
* ***a module is a single python file.***<br>
* ***a package is a folder containing python files, accompanied by a (can be empty) file named __init__.py, to tell python it is a package to import modules from.***<br>
## To find out what is included in $PYTHONPATH, (Python looks for its modules and packages in $PYTHONPATH)
```
import sys
print(sys.path)
```
## How to add a directory
* **add path(s) occasionally to the default path by insert operation**<br>
```
import sys
sys.path.insert(0, "/path/to/your/package_or_module")
```
    * **add a module(`module_1.py`) located in `/home/myname/pythonfiles` folder**<br>
    ```
    import sys
    # the following path of "/home/myname/pythonfiles/" can be replaced by relative path related to __file__
    sys.path.insert(0, "/home/myname/pythonfiles/")
    import module_1
    ```
    * **add a package(`my_package`) located in `/home/myname/pythonfiles` folder**<br>
        * **import a module named `module_2.py` located in `my_pakcage`**<br>
        * **import a module named `module_3.py` located in `subfolder` inside `my_package`**<br>
        ```
        # mypackage/module_2.py
        import sys
        # the following path of "/home/myname/pythonfiles/" can be replaced by relative path related to __file__
        sys.path.insert(0, "/home/myname/pythonfiles/")
        from my_package import module_2
        import my_package.subfolder.module_3
        ```
        ***Caution:***<br>
        * **Given the fact that all subfolders in the package include their own `__init__.py` file.**
    * **add a module which is in the same directory as the script or application**<br>
    ```
    import module_4
    ```

