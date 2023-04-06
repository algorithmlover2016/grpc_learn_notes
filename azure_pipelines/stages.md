## [stage](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/stages?view=azure-devops&tabs=yaml)
* When you define multiple stages in a pipeline, by default, they run one after the other.<br>
* By default, a stage runs if it doesn't depend on any other stage, or if all of the stages that it depends on have completed and succeeded. <br>
* it's common to use `and(succeeded(),custom_condition)` to check whether the preceding stage ran successfully. Otherwise, the stage runs regardless of the outcome of the preceding stage.<br>
```yml
stages:
- stage: A
  pool: StageAPool
  jobs:
  - job: A1 # will run on "StageAPool" pool based on the pool defined on the stage
  - job: A2 # will run on "JobPool" pool
    pool: JobPool
```

```yml
stages:
- stage: string  # name of the stage, A-Z, a-z, 0-9, and underscore
  displayName: string  # friendly name to display in the UI
  dependsOn: string | [ string ]
  condition: string
  pool: string | pool
  variables: { string: string } | [ variable | variableReference ] 
  jobs: [ job | templateReference]
```
```yml
# Example of fan-out and fan-in
stages:
- stage: Test

- stage: DeployUS1
  dependsOn: Test    # this stage runs after Test

- stage: DeployUS2
  dependsOn: Test    # this stage runs in parallel with DeployUS1, after Test

- stage: DeployEurope
  dependsOn:         # this stage runs after DeployUS1 and DeployUS2
  - DeployUS1
  - DeployUS2

# conditions
stages:
- stage: A

- stage: B
  condition: and(succeeded(), eq(variables['build.sourceBranch'], 'refs/heads/main'))
```
