## [container]()
```yml
# Only windows-2019 and ubuntu-* images support running containers.

# linux
pool:
  vmImage: 'ubuntu-latest'

container: ubuntu:18.04

steps:
- script: printenv

# window
pool:
  vmImage: 'windows-2019'

container: mcr.microsoft.com/windows/servercore:ltsc2019

steps:
- script: set

# multiple
pool:
  vmImage: 'ubuntu-latest'

strategy:
  matrix:
    ubuntu16:
      containerImage: ubuntu:16.04
    ubuntu18:
      containerImage: ubuntu:18.04
    ubuntu20:
      containerImage: ubuntu:20.04

container: $[ variables['containerImage'] ]

steps:
- script: printenv
```
### [endpoints](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/container-phases?view=azure-devops#endpoints)
```yml
container:
  image: registry:ubuntu1804
  endpoint: private_dockerhub_connection

steps:
- script: echo hello

# or
container:
  image: myprivate.azurecr.io/windowsservercore:1803
  endpoint: my_acr_connection

steps:
- script: echo hello

# options
container:
  image: ubuntu:18.04
  options: --hostname container-test --ip 192.168.0.1

steps:
- script: echo hello

# reusable
resources:
  containers:
  - container: u16
    image: ubuntu:16.04

  - container: u18
    image: ubuntu:18.04

  - container: u20
    image: ubuntu:20.04

jobs:
- job: RunInContainer
  pool:
    vmImage: 'ubuntu-latest'

  strategy:
    matrix:
      ubuntu16:
        containerResource: u16
      ubuntu18:
        containerResource: u18
      ubuntu20:
        containerResource: u20

  container: $[ variables['containerResource'] ]

  steps:
  - script: printenv
```