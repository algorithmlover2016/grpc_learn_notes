## [task](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/tasks?view=azure-devops&tabs=yaml)
* tasks running sequence in a job<br>
### [buildin tasks](https://learn.microsoft.com/en-us/azure/devops/pipelines/tasks/reference/?view=azure-pipelines&viewFallbackFrom=azure-devops)
### [custom tasks](https://learn.microsoft.com/en-us/azure/devops/extend/develop/add-build-task?view=azure-devops)
```yml
steps:
# You can set which minor version gets used by specifying the full version number of a task after the @ sign
- task: GoTool@0.3.1
# In YAML, you specify the major version using @ in the task name.
- task: myPublisherId.myExtensionId.myContributionId.myTaskName@1 #format example
- task: qetza.replacetokens.replacetokens-task.replacetokens@3 #working example
```
### [task control](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/tasks?view=azure-devops&tabs=yaml#task-control-options)
#### grammar syntax
```yml
- task: string  # reference to a task and version, e.g. "VSBuild@1"
  condition: expression     # see below
  continueOnError: boolean  # 'true' if future steps should run even if this step fails; defaults to 'false'
  enabled: boolean          # whether or not to run this step; defaults to 'true'
  retryCountOnTaskFailure: number # Max number of retries; default is zero
  timeoutInMinutes: number  # how long to wait before timing out the task
  target: string            # 'host' or the name of a container resource to target
```
* [condition](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/tasks?view=azure-devops&tabs=yaml#conditions)
```yml
steps:
# Only when all previous direct and indirect dependencies with the same agent pool have succeeded. This is the default if there is not a condition set in the YAML.
- task: UsePythonVersion@0
  inputs:
    versionSpec: '3.x'
    architecture: 'x64'

- task: PublishTestResults@2
  inputs:
    testResultsFiles: "**/TEST-*.xml"
  condition: succeededOrFailed() # PublishTestResults@2 will run even if the previous step fails, unless the run was canceled.
  # condition: always() # PublishTestResults@2 will run even if the previous step fails, even if the run was canceled.
  # condition: failed() # PublishTestResults@2 will run Only when a previous dependency has failed.
```
* [set step target](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/tasks?view=azure-devops&tabs=yaml#step-target)
```yml
resources:
  containers:
  - container: pycontainer
    image: python:3.11

steps:
# the SampleTask runs on the host and AnotherTask runs in a container.
- task: SampleTask@1
  target: host
- task: AnotherTask@1
  target: pycontainer
```
* [Environment variables](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/tasks?view=azure-devops&tabs=yaml#environment-variables)
    * Each task has an `env` property that is a list of string pairs that represent environment variables mapped into the task process.<br>
```yml
task: AzureCLI@2
displayName: Azure CLI
inputs: # Specific to each task
env:
  ENV_VARIABLE_NAME: value
  ENV_VARIABLE_NAME2: value
  ...
```
```yml
# Using the script shortcut syntax
- script: az pipelines variable-group list --output table
  env:
    AZURE_DEVOPS_EXT_PAT: $(System.AccessToken)
  displayName: 'List variable groups using the script step'

# Using the task syntax
- task: CmdLine@2
  inputs:
    script: az pipelines variable-group list --output table
  env:
    AZURE_DEVOPS_EXT_PAT: $(System.AccessToken)
  displayName: 'List variable groups using the command line task'
```
```yml
pool:
  vmImage: ubuntu-latest

steps:
# Node install
- task: NodeTool@0
  displayName: Node install
  inputs:
    versionSpec: '12.x' # The version we're installing
# Write the installed version to the command line
- script: which node
```

### [template](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/templates)
#### [Parameters](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/templates?view=azure-devops#parameters)
```yml
# File: simple-param.yml
parameters:
- name: yesNo # name of the parameter; required
  type: boolean # data type of the parameter; required
  default: false

steps:
- script: echo ${{ parameters.yesNo }}

# File: azure-pipelines.yml
trigger:
- main

extends:
  template: simple-param.yml
  parameters:
      yesNo: false # set to a non-boolean value to have the build fail
```

```yml
#azure-pipeline.yml
parameters:
- name: experimentalTemplate
  displayName: 'Use experimental build process?'
  type: boolean
  default: false

steps:
- ${{ if eq(parameters.experimentalTemplate, true) }}:
  - template: experimental.yml
- ${{ if not(eq(parameters.experimentalTemplate, true)) }}:
  - template: stable.yml
```

```yml
parameters:
- name: myString
  type: string
  default: a string
- name: myMultiString
  type: string
  default: default
  values:
  - default
  - ubuntu
- name: myNumber
  type: number
  default: 2
  values:
  - 1
  - 2
  - 4
  - 8
  - 16
- name: myBoolean
  type: boolean
  default: true
- name: myObject
  type: object
  default:
    foo: FOO
    bar: BAR
    things:
    - one
    - two
    - three
    nested:
      one: apple
      two: pear
      count: 3
- name: myStep
  type: step
  default:
    script: echo my step
- name: mySteplist
  type: stepList
  default:
    - script: echo step one
    - script: echo step two

trigger: none

jobs: 
- job: stepList
  steps: ${{ parameters.mySteplist }}
- job: myStep
  steps:
    - ${{ parameters.myStep }}
```
#### [iterable](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/templates?view=azure-devops#parameter-data-types)
```yml
parameters:
- name: listOfStrings
  type: object
  default:
  - one
  - two

steps:
- ${{ each value in parameters.listOfStrings }}:
  - script: echo ${{ value }}
```
```yml
parameters:
- name: listOfFruits
  type: object
  default:
  - fruitName: 'apple'
    colors: ['red','green']
  - fruitName: 'lemon'
    colors: ['yellow']

steps:
- ${{ each fruit in parameters.listOfFruits }} :
  - ${{ each fruitColor in fruit.colors}} :
    - script: echo ${{ fruit.fruitName}} ${{ fruitColor }}
```
```yml
# File: start.yml
parameters:
- name: buildSteps # the name of the parameter is buildSteps
  type: stepList # data type is StepList
  default: [] # default value of buildSteps
stages:
- stage: secure_buildstage
  pool:
    vmImage: windows-latest
  jobs:
  - job: secure_buildjob
    steps:
    - script: echo This happens before code 
      displayName: 'Base: Pre-build'
    - script: echo Building
      displayName: 'Base: Build'

    - ${{ each step in parameters.buildSteps }}:
      - ${{ each pair in step }}:
          ${{ if ne(pair.value, 'CmdLine@2') }}:
            ${{ pair.key }}: ${{ pair.value }}
          ${{ if eq(pair.value, 'CmdLine@2') }}:
            # Step is rejected by raising a YAML syntax error: Unexpected value 'CmdLine@2'
            '${{ pair.value }}': error         

    - script: echo This happens after code
      displayName: 'Base: Signing'

# File: azure-pipelines.yml
trigger:
- main

extends:
  template: start.yml
  parameters:
    buildSteps:
      - bash: echo Test #Passes
        displayName: succeed
      - bash: echo "Test"
        displayName: succeed
      # Step is rejected by raising a YAML syntax error: Unexpected value 'CmdLine@2'
      - task: CmdLine@2
        inputs:
          script: echo "Script Test"
      # Step is rejected by raising a YAML syntax error: Unexpected value 'CmdLine@2'
      - script: echo "Script Test"
```
```yml
#testing-template.yml
parameters: 
- name: testSet
  type: jobList

jobs:
- ${{ each testJob in parameters.testSet }}:
  - ${{ if eq(testJob.templateContext.expectedHTTPResponseCode, 200) }}:
    - job:
      steps: 
      - powershell: 'Invoke-RestMethod -Uri https://blogs.msdn.microsoft.com/powershell/feed/ | Format-Table -Property Title, pubDate'
      - ${{ testJob.steps }}    
  - ${{ if eq(testJob.templateContext.expectedHTTPResponseCode, 500) }}:
    - job:
      steps:
      - powershell: 'Get-ChildItem -Path Env:\'
      - ${{ testJob.steps }}


#azure-pipeline.yml
trigger: none

pool:
  vmImage: ubuntu-latest

extends:
  template: testing-template.yml
  parameters:
    testSet:
    - job: positive_test
      templateContext:
        expectedHTTPResponseCode: 200
      steps:
      - script: echo "Run positive test" 
    - job: negative_test
      templateContext:
        expectedHTTPResponseCode: 500
      steps:
      - script: echo "Run negative test"
```

```yml
# File: templates/include-npm-steps.yml
steps:
- script: npm install
- script: yarn install
- script: npm run compile


# File: azure-pipelines.yml
jobs:
- job: Linux
  pool:
    vmImage: 'ubuntu-latest'
  steps:
  - template: templates/include-npm-steps.yml  # Template reference
- job: Windows
  pool:
    vmImage: 'windows-latest'
  steps:
  - template: templates/include-npm-steps.yml  # Template reference
```
```yml
# File: templates/npm-steps.yml
steps:
- script: npm install
- script: npm test

# File: azure-pipelines.yml

jobs:
- job: Linux
  pool:
    vmImage: 'ubuntu-latest'
  steps:
  - template: templates/npm-steps.yml  # Template reference

- job: macOS
  pool:
    vmImage: 'macOS-latest'
  steps:
  - template: templates/npm-steps.yml  # Template reference

- job: Windows
  pool:
    vmImage: 'windows-latest'
  steps:
  - script: echo This script runs before the template's steps, only on Windows.
  - template: templates/npm-steps.yml  # Template reference
  - script: echo This step runs after the template's steps.
```

```yml
# File: templates/jobs.yml
jobs:
- job: Ubuntu
  pool:
    vmImage: 'ubuntu-latest'
  steps:
  - bash: echo "Hello Ubuntu"

- job: Windows
  pool:
    vmImage: 'windows-latest'
  steps:
  - bash: echo "Hello Windows"

  # File: azure-pipelines.yml

jobs:
- template: templates/jobs.yml  # Template reference

# When working with multiple jobs, remember to remove the name of the job in the template file, so as to avoid conflict
# File: templates/jobs.yml
jobs:
- job: 
  pool:
    vmImage: 'ubuntu-latest'
  steps:
  - bash: echo "Hello Ubuntu"

- job:
  pool:
    vmImage: 'windows-latest'
  steps:
  - bash: echo "Hello Windows"

# File: azure-pipelines.yml

jobs:
- template: templates/jobs.yml  # Template reference
- template: templates/jobs.yml  # Template reference
- template: templates/jobs.yml  # Template reference
```

```yml
# File: templates/stages1.yml
stages:
- stage: Angular
  jobs:
  - job: angularinstall
    steps:
    - script: npm install angular

# File: templates/stages2.yml
stages:
- stage: Build
  jobs:
  - job: build
    steps:
    - script: npm run build

# File: azure-pipelines.yml
trigger:
- main

pool:
  vmImage: 'ubuntu-latest'

stages:
- stage: Install
  jobs: 
  - job: npminstall
    steps:
    - task: Npm@1
      inputs:
        command: 'install'
- template: templates/stages1.yml
- template: templates/stages2.yml
```

```yml
# File: templates/npm-with-params.yml
parameters:
- name: name  # defaults for any parameters that aren't specified
  default: ''
- name: vmImage
  default: ''

jobs:
- job: ${{ parameters.name }}
  pool: 
    vmImage: ${{ parameters.vmImage }}
  steps:
  - script: npm install
  - script: npm test


# When you consume the template in your pipeline, specify values for the template parameters.
# File: azure-pipelines.yml
jobs:
- template: templates/npm-with-params.yml  # Template reference
  parameters:
    name: Linux
    vmImage: 'ubuntu-latest'

- template: templates/npm-with-params.yml  # Template reference
  parameters:
    name: macOS
    vmImage: 'macOS-latest'

- template: templates/npm-with-params.yml  # Template reference
  parameters:
    name: Windows
    vmImage: 'windows-latest'
```

```yml
# File: templates/steps-with-params.yml
parameters:
- name: 'runExtendedTests'  # defaults for any parameters that aren't specified
  type: boolean
  default: false

steps:
- script: npm test
- ${{ if eq(parameters.runExtendedTests, true) }}:
  - script: npm test --extended


# File: azure-pipelines.yml
steps:
- script: npm install

- template: templates/steps-with-params.yml  # Template reference
  parameters:
    runExtendedTests: 'true'
```
* Scalar parameters without a specified type are treated as strings.<br>
    *  `eq(true, parameters['myparam'])` will return `true`, even if the myparam parameter is the word false, if myparam is not explicitly made boolean.<br>
    *  Non-empty strings are cast to true in a Boolean context. That expression could be rewritten to explicitly compare strings: `eq(parameters['myparam'], 'true')`.<br>

```yml
# azure-pipelines.yml
jobs:
- template: process.yml
  parameters:
    pool:   # this parameter is called `pool`
      vmImage: ubuntu-latest  # and it's a mapping rather than a string


# process.yml
parameters:
- name: 'pool'
  type: object
  default: {}
jobs:
- job: build
  pool: ${{ parameters.pool }}
```

#### [Variable](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/templates?view=azure-devops#variable-reuse)
* Use parameters instead of variables when you want to restrict type.<br>
```yml
# File: vars.yml
variables:
  favoriteVeggie: 'brussels sprouts'

# File: azure-pipelines.yml
variables:
- template: vars.yml  # Template reference

steps:
- script: echo My favorite vegetable is ${{ variables.favoriteVeggie }}.
```
```yml
# File: templates/package-release-with-params.yml
parameters:
- name: DIRECTORY 
  type: string
  default: "." # defaults for any parameters that specified with "." (current directory)

variables:
- name: RELEASE_COMMAND
  value: grep version ${{ parameters.DIRECTORY }}/package.json | awk -F \" '{print $4}'


# File: azure-pipelines.yml
variables: # Global variables
  - template: package-release-with-params.yml # Template reference
    parameters:
      DIRECTORY: "azure/checker"

pool:
  vmImage: 'ubuntu-latest'

stages:
- stage: Release_Stage 
  displayName: Release Version
  variables: # Stage variables
  - template: package-release-with-params.yml  # Template reference
    parameters:
      DIRECTORY: "azure/todo-list"
  jobs: 
  - job: A
    steps: 
    - bash: $(RELEASE_COMMAND) #output release command
```

#### [user other repositories](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/templates?view=azure-devops#use-other-repositories)
```yml
# Repo: Contoso/BuildTemplates
# File: common.yml
parameters:
- name: 'vmImage'
  default: 'ubuntu 16.04'
  type: string

jobs:
- job: Build
  pool:
    vmImage: ${{ parameters.vmImage }}
  steps:
  - script: npm install
  - script: npm test

# Repo: Contoso/LinuxProduct
# File: azure-pipelines.yml
resources:
  repositories:
    - repository: templates
      type: github
      name: Contoso/BuildTemplates

jobs:
- template: common.yml@templates  # Template reference

# Repo: Contoso/WindowsProduct
# File: azure-pipelines.yml
resources:
  repositories:
    - repository: templates
      type: github
      name: Contoso/BuildTemplates
      ref: refs/tags/v1.0 # optional ref to pin to

jobs:
- template: common.yml@templates  # Template reference
  parameters:
    vmImage: 'windows-latest'


# Repo: otherOrg/WindowsProduct
# File: azure-pipelines.yml
resources:
  repositories:
  - repository: templates
    name: Contoso/BuildTemplates
    endpoint: myServiceConnection # Azure DevOps service connection
jobs:
- template: common.yml@templates
```
* Notice:<br>
    *  For `type: github`, name is `<identity>/<repo>` as in the examples above.<br>
    *  For `type: git` (Azure Repos), name is `<project>/<repo>`.<br>
        *  If that project is in a separate Azure DevOps organization, you'll need to configure a `service connection` of type Azure Repos/Team Foundation Server with access to the project and include that in YAML<br>
    *  use a particular, fixed version of the template, be sure to pin to a ref<br>
        *  branches (`refs/heads/<name>`)<br>
        *  tags (`refs/tags/<name>`)<br>

```yml
# Repo: Contoso/Central
# File: template.yml
jobs:
- job: PreBuild
  steps: []

  # Template reference to the repo where this template was
  # included from - consumers of the template are expected
  # to provide a "BuildJobs.yml"
- template: BuildJobs.yml@self

- job: PostBuild
  steps: []


# Repo: Contoso/MyProduct
# File: azure-pipelines.yml
resources:
  repositories:
    - repository: templates
      type: git
      name: Contoso/Central

extends:
  template: template.yml@templates


# Repo: Contoso/MyProduct
# File: BuildJobs.yml
jobs:
- job: Build
  steps: []
```

### [template expression](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/templates?view=azure-devops#template-expressions)
* `${{ }}`<br>
```yml
# File: steps/msbuild.yml
parameters:
- name: 'solution'
  default: '**/*.sln'
  type: string

steps:
- task: msbuild@1
  inputs:
    solution: ${{ parameters['solution'] }}  # index syntax
- task: vstest@2
  inputs:
    solution: ${{ parameters.solution }}  # property dereference syntax

# File: azure-pipelines.yml
steps:
- template: steps/msbuild.yml
  parameters:
    solution: my.sln
```

* [Required parameters](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/templates?view=azure-devops#required-parameters)
```yml
# File: steps/msbuild.yml
parameters:
- name: 'solution'
  default: ''
  type: string

steps:
- bash: |
    if [ -z "$SOLUTION" ]; then
      echo "##vso[task.logissue type=error;]Missing template parameter \"solution\""
      echo "##vso[task.complete result=Failed;]"
    fi
  env:
    SOLUTION: ${{ parameters.solution }}
  displayName: Check for required parameters
- task: msbuild@1
  inputs:
    solution: ${{ parameters.solution }}
- task: vstest@2
  inputs:
    solution: ${{ parameters.solution }}

# File: azure-pipelines.yml
# This will fail since it doesn't set the "solution" parameter to anything,
# so the template will use its default of an empty string
steps:
- template: steps/msbuild.yml
```

* [insertion](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/templates?view=azure-devops#insertion)
```yml
# File: jobs/build.yml
parameters:
- name: 'preBuild'
  type: stepList
  default: []
- name: 'preTest'
  type: stepList
  default: []
- name: 'preSign'
  type: stepList
  default: []

jobs:
- job: Build
  pool:
    vmImage: 'windows-latest'
  steps:
  - script: cred-scan
  - ${{ parameters.preBuild }}
  - task: msbuild@1
  - ${{ parameters.preTest }}
  - task: vstest@2
  - ${{ parameters.preSign }}
  - script: sign


# File: .vsts.ci.yml
jobs:
- template: jobs/build.yml
  parameters:
    preBuild:
    - script: echo hello from pre-build
    preTest:
    - script: echo hello from pre-test
```
* When an array is inserted into an array, the nested array is flattened.<br>
* To insert into a mapping, use the special property `${{ insert }}`.<br>
```yml
# Default values
parameters:
- name: 'additionalVariables'
  type: object
  default: {}

jobs:
- job: build
  variables:
    configuration: debug
    arch: x86
    ${{ insert }}: ${{ parameters.additionalVariables }}
  steps:
  - task: msbuild@1
  - task: vstest@2


jobs:
- template: jobs/build.yml
  parameters:
    additionalVariables:
      TEST_SUITE: L0,L1
```
* [Conditional insertion](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/templates?view=azure-devops#conditional-insertion)
```yml
# File: steps/build.yml
parameters:
- name: 'toolset'
  default: msbuild
  type: string
  values:
  - msbuild
  - dotnet

steps:
# msbuild
- ${{ if eq(parameters.toolset, 'msbuild') }}:
  - task: msbuild@1
  - task: vstest@2

# dotnet
- ${{ if eq(parameters.toolset, 'dotnet') }}:
  - task: dotnet@1
    inputs:
      command: build
  - task: dotnet@1
    inputs:
      command: test


# File: azure-pipelines.yml
steps:
- template: steps/build.yml
  parameters:
    toolset: dotnet
```
```yml
# File: steps/build.yml

parameters:
- name: 'debug'
  type: boolean
  default: false

steps:
- script: tool
  env:
    ${{ if eq(parameters.debug, true) }}:
      TOOL_DEBUG: true
      TOOL_DEBUG_DIR: _dbg


steps:
- template: steps/build.yml
  parameters:
    debug: true
```

```yml
variables:
  - name: foo
    value: test

pool:
  vmImage: 'ubuntu-latest'

steps:
- script: echo "start" # always runs
- ${{ if eq(variables.foo, 'test') }}:
  - script: echo "this is a test" # runs when foo=test
```

* [Iterative insertion](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/templates?view=azure-devops#iterative-insertion)
```yml
# job.yml
parameters:
- name: 'jobs'
  type: jobList
  default: []

jobs:
- ${{ each job in parameters.jobs }}: # Each job
  - ${{ each pair in job }}:          # Insert all properties other than "steps"
      ${{ if ne(pair.key, 'steps') }}:
        ${{ pair.key }}: ${{ pair.value }}
    steps:                            # Wrap the steps
    - task: SetupMyBuildTools@1       # Pre steps
    - ${{ job.steps }}                # Users steps
    - task: PublishMyTelemetry@1      # Post steps
      condition: always()

    
# azure-pipelines.yml
jobs:
- template: job.yml
  parameters:
    jobs:
    - job: A
      steps:
      - script: echo This will get sandwiched between SetupMyBuildTools and PublishMyTelemetry.
    - job: B
      steps:
      - script: echo So will this!
```
```yml
# job.yml
parameters:
- name: 'jobs'
  type: jobList
  default: []

jobs:
- job: SomeSpecialTool                # Run your special tool in its own job first
  steps:
  - task: RunSpecialTool@1
- ${{ each job in parameters.jobs }}: # Then do each job
  - ${{ each pair in job }}:          # Insert all properties other than "dependsOn"
      ${{ if ne(pair.key, 'dependsOn') }}:
        ${{ pair.key }}: ${{ pair.value }}
    dependsOn:                        # Inject dependency
    - SomeSpecialTool
    - ${{ if job.dependsOn }}:
      - ${{ job.dependsOn }}


# azure-pipelines.yml
jobs:
- template: job.yml
  parameters:
    jobs:
    - job: A
      steps:
      - script: echo This job depends on SomeSpecialTool, even though it's not explicitly shown here.
    - job: B
      dependsOn:
      - A
      steps:
      - script: echo This job depends on both Job A and on SomeSpecialTool.
```
* [Escape a value](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/templates?view=azure-devops#escape-a-value)
    * escape a value that literally contains `${{`<br>
```yml
${{ 'my${{value' }}
${{ 'my${{value with a '' single quote too' }}
```