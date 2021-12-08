# [demo of setup.py by reading dependencies from requirements.txt](https://newbedev.com/how-to-write-setup-py-to-include-a-git-repository-as-a-dependency)

```
from setuptools import setup, find_packages
from os import path

loc = path.abspath(path.dirname(__file__))

with open(loc + '/requirements.txt') as f:
    requirements = f.read().splitlines()

required = []
dependency_links = []

# Do not add to required lines pointing to Git repositories
EGG_MARK = '#egg='
for line in requirements:
    if line.startswith('-e git:') or line.startswith('-e git+') or \
            line.startswith('git:') or line.startswith('git+'):
        line = line.lstrip('-e ')  # in case that is using "-e"
        if EGG_MARK in line:
            package_name = line[line.find(EGG_MARK) + len(EGG_MARK):]
            repository = line[:line.find(EGG_MARK)]
            required.append('%s @ %s' % (package_name, repository))
            dependency_links.append(line)
        else:
            print('Dependency to a git repository should have the format:')
            print('git+ssh://git@github.com/xxxxx/xxxxxx#egg=package_name')
    else:
        required.append(line)

setup(
    name='myproject',  # Required
    version='0.0.1',  # Required
    description='Description here....',  # Required
    packages=find_packages(),  # Required
    install_requires=required,
    dependency_links=dependency_links,
)
```
# [demo of setup.py by coding](https://stackoverflow.com/questions/15724093/difference-between-python-setup-py-install-and-pip-install/15731459#15731459)
* **[Differences between python setup.py install and pip install <package_name>](https://stackoverflow.com/questions/32688688/how-to-write-setup-py-to-include-a-git-repository-as-a-dependency)**<br>
```
from setuptools import setup, find_packages

setup(
    name = 'MyProject',
    version = '0.1.0',
    url = '',
    description = '',
    packages = find_packages(),
    install_requires = [
        # Github Private Repository
        'ExampleRepo @ git+ssh://git@github.com/example_org/ExampleRepo.git'
    ]
)
```