## [job](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/phases?view=azure-devops&tabs=yaml)

```yml
jobs:
- job: string  # name of the job, A-Z, a-z, 0-9, and underscore
  ...
  workspace:
    clean: outputs | resources | all # what to clean up before the job runs
```
* Notice:<br>
    * A job is a series of steps that run sequentially as a unit<br>
    * a job is the smallest unit of work that can be scheduled to run.<br>

### [noral job full syntax]()
```yml
# full syntax to specify a job
- job: string  # name of the job, A-Z, a-z, 0-9, and underscore
  displayName: string  # friendly name to display in the UI
  dependsOn: string | [ string ]
  condition: string
  strategy:
    parallel: # parallel strategy
    matrix: # matrix strategy
    maxParallel: number # maximum number simultaneous matrix legs to run
    # note: `parallel` and `matrix` are mutually exclusive
    # you may specify one or the other; including both is an error
    # `maxParallel` is only valid with `matrix`
  continueOnError: boolean  # 'true' if future jobs should run even if this job fails; defaults to 'false'
  pool: pool # agent pool
  workspace:
    clean: outputs | resources | all # what to clean up before the job runs
  container: containerReference # container to run this job inside
  timeoutInMinutes: number # how long to run the job before automatically cancelling
  cancelTimeoutInMinutes: number # how much time to give 'run always even if cancelled tasks' before killing them
  variables: { string: string } | [ variable | variableReference ] 
  steps: [ script | bash | pwsh | powershell | checkout | task | templateReference ]
  services: { string: string | container } # container resources to run as a service container
  uses: # Any resources (repos or pools) required by this job that are not already referenced
    repositories: [ string ] # Repository references to Azure Git repositories
    pools: [ string ] # Pool names, typically when using a matrix strategy for the job
```
### [deployment job full syntax](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/deployment-jobs?view=azure-devops#schema)
```yml
#  full syntax to specify a deployment job
jobs:
# instead of job keyword, use deployment keyword
- deployment: string   # name of the deployment job, A-Z, a-z, 0-9, and underscore. The word "deploy" is a keyword and is unsupported as the deployment name.
  displayName: string  # friendly name to display in the UI
  pool:                # not required for virtual machine resources
    name: string       # Use only global level variables for defining a pool name. Stage/job level variables are not supported to define pool name.
    demands: string | [ string ]
  workspace:
    clean: outputs | resources | all # what to clean up before the job runs
  dependsOn: string
  condition: string
  continueOnError: boolean                # 'true' if future jobs should run even if this job fails; defaults to 'false'
  container: containerReference # container to run this job inside
  services: { string: string | container } # container resources to run as a service container
  timeoutInMinutes: nonEmptyString        # how long to run the job before automatically cancelling
  cancelTimeoutInMinutes: nonEmptyString  # how much time to give 'run always even if cancelled tasks' before killing them
  variables: # several syntaxes, see specific section
  environment: string  # target environment name and optionally a resource name to record the deployment history; format: <environment-name>.<resource-name>
  # environment:
  #   name: string # Name of environment.
  #   resourceName: string # Name of resource.
  #   resourceId: string # Id of resource.
  #   resourceType: string # Type of environment resource.
  #   tags: string # List of tag filters.

  strategy:
    runOnce:    #rolling, canary are the other strategies that are supported
      deploy:
        steps: [ script | bash | pwsh | powershell | checkout | task | templateReference ]
```
#### [Descriptions of lifecycle hooks](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/deployment-jobs?view=azure-devops#descriptions-of-lifecycle-hooks)
```yml
# https://learn.microsoft.com/en-us/azure/devops/pipelines/process/deployment-jobs?view=azure-devops#runonce-deployment-strategy
strategy: 
    runOnce:
      preDeploy:        
        pool: [ server | pool ] # See pool schema.        
        steps:
        - script: [ script | bash | pwsh | powershell | checkout | task | templateReference ]
      deploy:          
        pool: [ server | pool ] # See pool schema.        
        steps:
        ...
      routeTraffic:         
        pool: [ server | pool ]         
        steps:
        ...        
      postRouteTraffic:          
        pool: [ server | pool ]        
        steps:
        ...
      on:
        failure:         
          pool: [ server | pool ]           
          steps:
          ...
        success:          
          pool: [ server | pool ]           
          steps:
          ...


# A rolling deployment replaces instances of the previous version of an application with instances of the new version of the application on a fixed set of virtual machines (rolling set) in each iteration.
# https://learn.microsoft.com/en-us/azure/devops/pipelines/process/deployment-jobs?view=azure-devops#rolling-deployment-strategy
strategy:
  rolling:
    maxParallel: [ number or percentage as x% ]
    preDeploy:        
      steps:
      - script: [ script | bash | pwsh | powershell | checkout | task | templateReference ]
    deploy:          
      steps:
      ...
    routeTraffic:         
      steps:
      ...        
    postRouteTraffic:          
      steps:
      ...
    on:
      failure:         
        steps:
        ...
      success:          
        steps:
        ...

# By using this strategy, you can roll out the changes to a small subset of servers first.
# As you gain more confidence in the new version, you can release it to more servers in your infrastructure and route more traffic to it.
strategy: 
    canary:
      increments: [ number ]
      preDeploy:        
        pool: [ server | pool ] # See pool schema.        
        steps:
        - script: [ script | bash | pwsh | powershell | checkout | task | templateReference ]
      deploy:          
        pool: [ server | pool ] # See pool schema.        
        steps:
        ...
      routeTraffic:         
        pool: [ server | pool ]         
        steps:
        ...        
      postRouteTraffic:          
        pool: [ server | pool ]        
        steps:
        ...
      on:
        failure:         
          pool: [ server | pool ]           
          steps:
          ...
        success:          
          pool: [ server | pool ]           
          steps:
          ...
```

### [Simple Jobs](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/phases?view=azure-devops&tabs=yaml#define-a-single-job)
```yml
pool:
  vmImage: 'ubuntu-latest'
steps:
- bash: echo "Hello world"

jobs:
- job: myJob
  timeoutInMinutes: 10
  pool:
    vmImage: 'ubuntu-latest'
  steps:
  - bash: echo "Hello world"

jobs:
- job: A
  steps:
  - bash: echo "A"

- job: B
  steps:
  - bash: echo "B"


stages:
- stage: A
  jobs:
  - job: A1
  - job: A2

- stage: B
  jobs:
  - job: B1
  - job: B2
```

### [Types of jobs](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/phases?view=azure-devops&tabs=yaml#types-of-jobs)
#### [Agent pool jobs](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/phases?view=azure-devops&tabs=yaml#agent-pool-jobs)
```yml
pool:
  name: myPrivateAgents    # your job runs on an agent in this pool
  demands: agent.os -equals Windows_NT    # the agent must have this capability to run the job
steps:
- script: echo hello world


pool:
  name: myPrivateAgents
  demands:
  - agent.os -equals Darwin
  - anotherCapability -equals somethingElse
steps:
- script: echo hello world
```

#### [server jobs](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/phases?view=azure-devops&tabs=yaml#server-jobs)
```yml
jobs:
- job: string
  timeoutInMinutes: number
  cancelTimeoutInMinutes: number
  strategy:
    maxParallel: number
    matrix: { string: { string: string } }

  pool: server # note: the value 'server' is a reserved keyword which indicates this is an agentless job
```

### [dependency jobs](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/phases?view=azure-devops&tabs=yaml#dependencies)
```yml
# The syntax for defining multiple jobs and their dependencies
jobs:
- job: string
  dependsOn: string
  condition: string


# build sequentially
jobs:
- job: Debug
  steps:
  - script: echo hello from the Debug build
- job: Release
  dependsOn: Debug
  steps:
  - script: echo hello from the Release build


# build in parallel
jobs:
- job: Windows
  pool:
    vmImage: 'windows-latest'
  steps:
  - script: echo hello from Windows
- job: macOS
  pool:
    vmImage: 'macOS-latest'
  steps:
  - script: echo hello from macOS
- job: Linux
  pool:
    vmImage: 'ubuntu-latest'
  steps:
  - script: echo hello from Linux

# fan-out
jobs:
- job: InitialJob
  steps:
  - script: echo hello from initial job
- job: SubsequentA
  dependsOn: InitialJob
  steps:
  - script: echo hello from subsequent A
- job: SubsequentB
  dependsOn: InitialJob
  steps:
  - script: echo hello from subsequent B

# fan-in
jobs:
- job: InitialA
  steps:
  - script: echo hello from initial A
- job: InitialB
  steps:
  - script: echo hello from initial B
- job: Subsequent
  dependsOn:
  - InitialA
  - InitialB
  steps:
  - script: echo hello from subsequent
```
#### [Stage to stage dependencies](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/expressions?view=azure-devops#dependencies)
```yml
"dependencies": {
  "<STAGE_NAME>" : {
    "result": "Succeeded|SucceededWithIssues|Skipped|Failed|Canceled",
    "outputs": {
        "jobName.stepName.variableName": "value"
    }
  },
  "...": {
    // another stage
  }
}
```
```yml
# Stage B runs whether Stage A is successful or skipped
stages:
- stage: A
  condition: false
  jobs:
  - job: A1
    steps:
    - script: echo Job A1
- stage: B
  condition: in(dependencies.A.result, 'Succeeded', 'SucceededWithIssues', 'Skipped')
  jobs:
  - job: B1
    steps:
    - script: echo Job B1


# Stage B depends on a variable in Stage A
stages:
- stage: A
  jobs:
  - job: A1
    steps:
     - bash: echo "##vso[task.setvariable variable=shouldrun;isOutput=true]true"
     # or on Windows:
     # - script: echo ##vso[task.setvariable variable=shouldrun;isOutput=true]true
       name: printvar

- stage: B
  condition: and(succeeded(), eq(dependencies.A.outputs['A1.printvar.shouldrun'], 'true'))
  dependsOn: A
  jobs:
  - job: B1
    steps:
    - script: echo hello from Stage B
```

#### [job to job within one stage](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/expressions?view=azure-devops#job-to-job-dependencies-within-one-stage)
```yml
"dependencies": {
  "<JOB_NAME>": {
    "result": "Succeeded|SucceededWithIssues|Skipped|Failed|Canceled",
    "outputs": {
      "stepName.variableName": "value1"
    }
  },
  "...": {
    // another job
  }
}
```
```yml
# Job A will always be skipped and Job B will run. Job C will run, since all of its dependencies either succeed or are skipped.
jobs:
- job: a
  condition: false
  steps:
  - script: echo Job A
- job: b
  steps:
  - script: echo Job B
- job: c
  dependsOn:
  - a
  - b
  condition: |
    and
    (
      in(dependencies.a.result, 'Succeeded', 'SucceededWithIssues', 'Skipped'),
      in(dependencies.b.result, 'Succeeded', 'SucceededWithIssues', 'Skipped')
    )
  steps:
  - script: echo Job C


# Job B depends on an output variable from Job A
jobs:
- job: A
  steps:
  - bash: echo "##vso[task.setvariable variable=shouldrun;isOutput=true]true"
  # or on Windows:
  # - script: echo ##vso[task.setvariable variable=shouldrun;isOutput=true]true
    name: printvar

- job: B
  condition: and(succeeded(), eq(dependencies.A.outputs['printvar.shouldrun'], 'true'))
  dependsOn: A
  steps:
  - script: echo hello from B
```
#### [job to job dependencies across stages](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/expressions?view=azure-devops#job-to-job-dependencies-across-stages)
```yml
"stageDependencies": {
  "<STAGE_NAME>" : {
    "<JOB_NAME>": {
      "result": "Succeeded|SucceededWithIssues|Skipped|Failed|Canceled",
      "outputs": {
          "stepName.variableName": "value"
      }
    },
    "...": {
      // another job
    }
  },
  "...": {
    // another stage
  }
}
```
```yml
# job B1 will run if job A1 is skipped. Job B2 will check the value of the output variable from job A1 to determine whether it should run
trigger: none

pool:
  vmImage: 'ubuntu-latest'

stages:
- stage: A
  jobs:
  - job: A1
    steps:
     - bash: echo "##vso[task.setvariable variable=shouldrun;isOutput=true]true"
     # or on Windows:
     # - script: echo ##vso[task.setvariable variable=shouldrun;isOutput=true]true
       name: printvar

- stage: B
  dependsOn: A
  jobs:
  - job: B1
    condition: in(stageDependencies.A.A1.result, 'Skipped') # change condition to `Succeeded and stage will be skipped`
    steps:
    - script: echo hello from Job B1
  - job: B2
    condition: eq(stageDependencies.A.A1.outputs['printvar.shouldrun'], 'true')
    steps:
     - script: echo hello from Job B2
```
* If a job depends on a variable defined by a deployment job in a different stage, then the syntax is different<br>
```yml
stages:
- stage: build
  jobs:
  - deployment: build_job
    environment:
      name: Production
    strategy:
      runOnce:
        deploy:
          steps:
          - task: PowerShell@2
            name: setRunTests
            inputs:
              targetType: inline
              pwsh: true
              script: |
                $runTests = "true"
                echo "setting runTests: $runTests"
                echo "##vso[task.setvariable variable=runTests;isOutput=true]$runTests"

- stage: test
  dependsOn:
  - 'build'
  jobs:  
    - job: run_tests
      # Notice that the key used for the outputs dictionary is build_job.setRunTests.runTests.<br>
      condition: eq(stageDependencies.build.build_job.outputs['build_job.setRunTests.runTests'], 'true')
      steps:
        ...
```
#### [stage depending on job ouput](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/expressions?view=azure-devops#stage-depending-on-job-output)
```yml
# The following example is a simple script that sets a variable (use your actual information from Terraform Plan) in a step in a stage,
# and then invokes the second stage only if the variable has a specific value.
stages:
- stage: plan_dev
  jobs:
  - job: terraform_plan_dev
    steps:
    - bash: echo '##vso[task.setvariable variable=terraform_plan_exitcode;isOutput=true]2'
      name: terraform_plan
- stage: apply_dev
  dependsOn: plan_dev
  condition: eq(dependencies.plan_dev.outputs['terraform_plan_dev.terraform_plan.terraform_plan_exitcode'], '2')
  jobs:
  - job: part_b
    steps:
    - bash: echo "BA"
```
* If a stage depends on a variable defined by a deployment job in a different stage, then the syntax is different.<br>
```yml
# Notice that in the condition of the test stage, build_job appears twice.
stages:
- stage: build
  jobs:
  - deployment: build_job
    environment:
      name: Production
    strategy:
      runOnce:
        deploy:
          steps:
          - task: PowerShell@2
            name: setRunTests
            inputs:
              targetType: inline
              pwsh: true
              script: |
                $runTests = "true"
                echo "setting runTests: $runTests"
                echo "##vso[task.setvariable variable=runTests;isOutput=true]$runTests"

- stage: test
  dependsOn:
  - 'build'
  condition: eq(dependencies.build.outputs['build_job.build_job.setRunTests.runTests'], 'true')
  jobs:
    ...
```
* To reference an environment resource, you'll need to add the environment resource name to the dependencies condition.<br>
```yml
stages:
- stage: build
  jobs:
  - deployment: build_job
    environment:
      name: vmtest
      resourceName: winVM2
      resourceType: VirtualMachine
    strategy:
      runOnce:
        deploy:
          steps:
          - task: PowerShell@2
            name: setRunTests
            inputs:
              targetType: inline
              pwsh: true
              script: |
                $runTests = "true"
                echo "setting runTests: $runTests"
                echo "##vso[task.setvariable variable=runTests;isOutput=true]$runTests"

- stage: test
  dependsOn:
  - 'build'
  condition: eq(dependencies.build.outputs['build_job.Deploy_winVM2.setRunTests.runTests'], 'true')
  jobs:
    ...
```


### [Conditions](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/phases?view=azure-devops&tabs=yaml#conditions)
```yml
# run a job based upon the status of running a previous job
jobs:
- job: A
  steps:
  - script: exit 1

- job: B
  dependsOn: A
  condition: failed()
  steps:
  - script: echo this will run when A fails

- job: C
  dependsOn:
  - A
  - B
  condition: succeeded('B')
  steps:
  - script: echo this will run when B runs and succeeds


# using a custom condition
jobs:
- job: A
  steps:
  - script: echo hello

- job: B
  dependsOn: A
  condition: and(succeeded(), eq(variables['build.sourceBranch'], 'refs/heads/master'))
  steps:
  - script: echo this only runs for master

# a job run based on the value of an output variable set in a previous job
# In this case, you can only use variables set in directly dependent jobs
jobs:
- job: A
  steps:
  - script: "echo '##vso[task.setvariable variable=skipsubsequent;isOutput=true]false'"
    name: printvar

- job: B
  condition: and(succeeded(), ne(dependencies.A.outputs['printvar.skipsubsequent'], 'true'))
  dependsOn: A
  steps:
  - script: echo hello from B
```

### [multi-job config](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/phases?view=azure-devops&tabs=yaml#multi-job-configuration)
* The `matrix` strategy enables a job to be dispatched multiple times, with different variable sets.
* The `maxParallel` tag restricts the amount of parallelism. <br>
```yml
jobs:
- job: Test
  strategy:
    maxParallel: 2
    matrix: 
      US_IE:
        Location: US
        Browser: IE
      US_Chrome:
        Location: US
        Browser: Chrome
      Europe_Chrome:
        Location: Europe
        Browser: Chrome
```
* If you want to make a variable available to future jobs, you must mark it as an output variable by using `isOutput=true`.
* Then you can map it into future jobs by using the `$[]` syntax and including the step name that set the variable.
* Multi-job output variables only work for jobs in the same stage.
```yml
jobs:
# Set an output variable from job A
- job: A
  pool:
    vmImage: 'windows-latest'
  steps:
  - powershell: echo "##vso[task.setvariable variable=myOutputVar;isOutput=true]this is the value"
    name: setvarStep
  - script: echo $(setvarStep.myOutputVar)
    name: echovar

# Map the variable into job B
- job: B
  dependsOn: A
  pool:
    vmImage: 'ubuntu-18.04'
  variables:
    # $[ dependencies.A.outputs['setvarStep.myOutputVar'] ] is assigned to the variable $(myVarFromJobA).
    myVarFromJobA: $[ dependencies.A.outputs['setvarStep.myOutputVar'] ]  # map in the variable
                                                                          # remember, expressions require single quotes
  steps:
  - script: echo $(myVarFromJobA)
    name: echovar
```
* setting a variable from one stage to another, use `stageDependencies`<br>
```yml
stages:
- stage: A
  jobs:
  - job: A1
    steps:
     - bash: echo "##vso[task.setvariable variable=myStageOutputVar;isOutput=true]this is a stage output var"
       name: printvar

- stage: B
  dependsOn: A
  variables:
    myVarfromStageA: $[ stageDependencies.A.A1.outputs['printvar.myStageOutputVar'] ]
  jobs:
  - job: B1
    steps:
    - script: echo $(myVarfromStageA)
```

```yml
# matrix
jobs:
# Set an output variable from a job with a matrix
- job: A
  pool:
    vmImage: 'ubuntu-18.04'
  strategy:
    maxParallel: 2
    matrix:
      debugJob:
        configuration: debug
        platform: x64
      releaseJob:
        configuration: release
        platform: x64
  steps:
  - bash: echo "##vso[task.setvariable variable=myOutputVar;isOutput=true]this is the $(configuration) value"
    name: setvarStep
  - bash: echo $(setvarStep.myOutputVar)
    name: echovar

# Map the variable from the debug job
- job: B
  dependsOn: A
  pool:
    vmImage: 'ubuntu-18.04'
  variables:
    myVarFromJobADebug: $[ dependencies.A.outputs['debugJob.setvarStep.myOutputVar'] ]
  steps:
  - script: echo $(myVarFromJobADebug)
    name: echovar

# slicing
jobs:
# Set an output variable from a job with slicing
- job: A
  pool:
    vmImage: 'ubuntu-18.04'
    parallel: 2 # Two slices
  steps:
  - bash: echo "##vso[task.setvariable variable=myOutputVar;isOutput=true]this is the slice $(system.jobPositionInPhase) value"
    name: setvarStep
  - script: echo $(setvarStep.myOutputVar)
    name: echovar

# Map the variable from the job for the first slice
- job: B
  dependsOn: A
  pool:
    vmImage: 'ubuntu-18.04'
  variables:
    myVarFromJobsA1: $[ dependencies.A.outputs['job1.setvarStep.myOutputVar'] ]
  steps:
  - script: "echo $(myVarFromJobsA1)"
    name: echovar

# deployment
jobs:
# Set an output variable from a deployment
- deployment: A
  pool:
    vmImage: 'ubuntu-18.04'
  environment: staging
  strategy:
    runOnce:
      deploy:
        steps:
        - bash: echo "##vso[task.setvariable variable=myOutputVar;isOutput=true]this is the deployment variable value"
          name: setvarStep
        - bash: echo $(setvarStep.myOutputVar)
          name: echovar

# Map the variable from the job for the first slice
- job: B
  dependsOn: A
  pool:
    vmImage: 'ubuntu-18.04'
  variables:
    myVarFromDeploymentJob: $[ dependencies.A.outputs['A.setvarStep.myOutputVar'] ]
  steps:
  - bash: "echo $(myVarFromDeploymentJob)"
    name: echovar
```

### [job variables](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/phases?view=azure-devops&tabs=yaml#job-variables)
```yml
variables:
  mySimpleVar: simple var value
  "my.dotted.var": dotted var value
  "my var with spaces": var with spaces value

steps:
- script: echo Input macro = $(mySimpleVar). Env var = %MYSIMPLEVAR%
  condition: eq(variables['agent.os'], 'Windows_NT')
- script: echo Input macro = $(mySimpleVar). Env var = $MYSIMPLEVAR
  condition: in(variables['agent.os'], 'Darwin', 'Linux')
- bash: echo Input macro = $(my.dotted.var). Env var = $MY_DOTTED_VAR
- powershell: Write-Host "Input macro = $(my var with spaces). Env var = $env:MY_VAR_WITH_SPACES"
```

### [workspace](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/phases?view=azure-devops&tabs=yaml#workspace)
* `Pipeline.Workspace`<br>
* `Build.SourcesDirectory`<br>
* `Build.ArtifactStagingDirectory`<br>
* `Build.BinariesDirectory`<br>
* `Common.TestResultsDirectory`<br>

* The `$(Build.ArtifactStagingDirectory)` and `$(Common.TestResultsDirectory)` are always deleted and recreated prior to every build.
  * When you specify one of the `clean` options, they are interpreted as follows:<br>
    * outputs: Delete `Build.BinariesDirectory` before running a new job.<br>
    * resources: Delete `Build.SourcesDirectory` before running a new job.<br>
    * all: Delete the entire `Pipeline.Workspace` directory before running a new job.<br>
```yml
# test and upload my code as an artifact named WebSite
jobs:
- job: Build
  pool:
    vmImage: 'ubuntu-latest'
  steps:
  - script: npm test
  - task: PublishBuildArtifacts@1
    inputs:
      pathtoPublish: '$(System.DefaultWorkingDirectory)'
      artifactName: WebSite

# download the artifact and deploy it only if the build job succeeded
- job: Deploy
  pool:
    vmImage: 'ubuntu-latest'
  steps:
  - checkout: none #skip checking out the default repository resource
  - task: DownloadBuildArtifacts@0
    displayName: 'Download Build Artifacts'
    inputs:
      artifactName: WebSite
      downloadPath: $(System.DefaultWorkingDirectory) # $(Pipeline.Workspace)

  dependsOn: Build
  condition: succeeded()
  ```

  ```yml
  steps:
- powershell: |
    $url = "$($env:SYSTEM_TEAMFOUNDATIONCOLLECTIONURI)$env:SYSTEM_TEAMPROJECTID/_apis/build/definitions/$($env:SYSTEM_DEFINITIONID)?api-version=4.1-preview"
    Write-Host "URL: $url"
    $pipeline = Invoke-RestMethod -Uri $url -Headers @{
      Authorization = "Bearer $env:SYSTEM_ACCESSTOKEN"
    }
    Write-Host "Pipeline = $($pipeline | ConvertTo-Json -Depth 100)"
  env:
    SYSTEM_ACCESSTOKEN: $(system.accesstoken)
  ```


  ```yml
  # RunOnce deployment strategy https://learn.microsoft.com/en-us/azure/devops/pipelines/process/deployment-jobs?view=azure-devops#runonce-deployment-strategy-1
  jobs:
  # Track deployments on the environment.
- deployment: DeployWeb
  displayName: deploy Web App
  pool:
    vmImage: 'ubuntu-latest'
  # Creates an environment if it doesn't exist.
  environment: 'smarthotel-dev'
  strategy:
    # Default deployment strategy, more coming...
    runOnce:
      deploy:
        steps:
        - checkout: self 
        - script: echo my first deployment

jobs:
- deployment: DeployWeb
  displayName: deploy Web App
  pool:
    vmImage: 'ubuntu-latest'
  # Records deployment against bookings resource - Kubernetes namespace.
  environment: 'smarthotel-dev.bookings'
  strategy: 
    runOnce:
      deploy:
        steps:
          # No need to explicitly pass the connection details.
        - task: KubernetesManifest@0
          displayName: Deploy to Kubernetes cluster
          inputs:
            action: deploy
            namespace: $(k8sNamespace)
            manifests: |
              $(System.ArtifactsDirectory)/manifests/*
            imagePullSecrets: |
              $(imagePullSecret)
            containers: |
              $(containerRegistry)/$(imageRepository):$(tag)
  
  # Rolling deployment strategy
  jobs: 
- deployment: VMDeploy
  displayName: web
  environment:
    name: smarthotel-dev
    resourceType: VirtualMachine
  strategy:
    rolling:
      maxParallel: 5  #for percentages, mention as x%
      preDeploy:
        steps:
        - download: current
          artifact: drop
        - script: echo initialize, cleanup, backup, install certs
      deploy:
        steps:
        - task: IISWebAppDeploymentOnMachineGroup@0
          displayName: 'Deploy application to Website'
          inputs:
            WebSiteName: 'Default Web Site'
            Package: '$(Pipeline.Workspace)/drop/**/*.zip'
      routeTraffic:
        steps:
        - script: echo routing traffic
      postRouteTraffic:
        steps:
        - script: echo health check post-route traffic
      on:
        failure:
          steps:
          - script: echo Restore from backup! This is on failure
        success:
          steps:
          - script: echo Notify! This is on success

# Canary deployment strategy
jobs: 
- deployment: 
  environment: smarthotel-dev.bookings
  pool: 
    name: smarthotel-devPool
  strategy:                  
    canary:      
      increments: [10,20]  
      preDeploy:                                     
        steps:           
        - script: initialize, cleanup....   
      deploy:             
        steps: 
        - script: echo deploy updates... 
        - task: KubernetesManifest@0 
          inputs: 
            action: $(strategy.action)       
            namespace: 'default' 
            strategy: $(strategy.name) 
            percentage: $(strategy.increment) 
            manifests: 'manifest.yml' 
      postRouteTraffic: 
        pool: server 
        steps:           
        - script: echo monitor application health...   
      on: 
        failure: 
          steps: 
          - script: echo clean-up, rollback...   
        success: 
          steps: 
          - script: echo checks passed, notify...
  ```

  ```yml
  stages:
- stage: StageA
  jobs:
  - deployment: A1
    pool:
      vmImage: 'ubuntu-latest'
    environment: env1
    strategy:                  
      runOnce:
        deploy:
          steps:
          - bash: echo "##vso[task.setvariable variable=myOutputVar;isOutput=true]this is the deployment variable value"
            name: setvarStep
          - bash: echo $(System.JobName)
  - deployment: A2
    pool:
      vmImage: 'ubuntu-latest'
    environment: 
      name: env2
      resourceName: vmsfortesting
      resourceType: virtualmachine
    strategy:                  
      runOnce:
        deploy:
          steps:
          - script: echo "##vso[task.setvariable variable=myOutputVarTwo;isOutput=true]this is the second deployment variable value"
            name: setvarStepTwo
  
  - job: B1
    dependsOn: A1
    pool:
      vmImage: 'ubuntu-latest'
    variables:
      myVarFromDeploymentJob: $[ dependencies.A1.outputs['A1.setvarStep.myOutputVar'] ]
      
    steps:
    - script: "echo $(myVarFromDeploymentJob)"
      name: echovar
 
  - job: B2
    dependsOn: A2
    pool:
      vmImage: 'ubuntu-latest'
    variables:
      myVarFromDeploymentJob: $[ dependencies.A2.outputs['A2.setvarStepTwo.myOutputVarTwo'] ]
      myOutputVarTwo: $[ dependencies.A2.outputs['Deploy_vmsfortesting.setvarStepTwo.myOutputVarTwo'] ]
    
    steps:
    - script: "echo $(myOutputVarTwo)"
      name: echovartwo
  ```