## [customize CI trigger](https://learn.microsoft.com/en-us/azure/devops/pipelines/customize-pipeline?view=azure-devops#customize-ci-triggers)
```yml
trigger:
  - main
  - releases/*
```
* For a pull request validation trigger<br>
```yml
pr:
  - main
  - releases/*
```
### [CI triggers](https://learn.microsoft.com/en-us/azure/devops/pipelines/repos/azure-repos-git?view=azure-devops&tabs=yaml#ci-triggers)
* Control Branch<br>
```yml
trigger:
- master # the full name of the branch
- releases/* # a wildcard (https://learn.microsoft.com/en-us/azure/devops/pipelines/repos/azure-repos-git?view=azure-devops&tabs=yaml#wildcards)
```
```yml
# wildcard
trigger:
  branches:
    include:
    - master
    - releases/*
    - feature/*
    exclude:
    - releases/old*
    - feature/*-working
  paths:
    include:
    - docs/*.md
```

```yml
trigger:
  branches:
    include:
      - refs/tags/{tagname}
    exclude:
      - refs/tags/{othertagname}
```

```yml
trigger:
  branches:
    include:
    - '*'  # must quote since "*" is a YAML reserved character; we want a string
```
* Batching CI<br>
  * If you set batch to true, when a pipeline is running, the system waits until the run is completed, then starts another run with all changes that have not yet been built.<br>
  * If the pipeline has multiple jobs and stages, then the first run should still reach a terminal state by completing or skipping all its jobs and stages before the second run can start.<br>
```yml
# specific branch build with batching
trigger:
  batch: true
  branches:
    include:
    - master
```

* [Path](https://learn.microsoft.com/en-us/azure/devops/pipelines/repos/azure-repos-git?view=azure-devops&tabs=yaml#paths)
```yml
# specific path build
trigger:
  batch: true
  branches:
    include:
    - master
    - releases/*
  paths:
    include:
    - docs
    exclude:
    - docs/README.md
```

* [Tags](https://learn.microsoft.com/en-us/azure/devops/pipelines/repos/azure-repos-git?view=azure-devops&tabs=yaml#tags)
  * If you don't specify any tag triggers, then by default, tags will not trigger pipelines.<br>
```yml
# specific tag
trigger:
  tags:
    include:
    - v2.*
    exclude:
    - v2.0
```

* [Scheduled triggers](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/scheduled-triggers?view=azure-devops&tabs=yaml#scheduled-triggers)
```yml
schedules:
- cron: string # cron syntax defining a schedulem, The time zone for cron schedules is UTC. 
  displayName: string # friendly name given to a specific schedule
  branches:
    include: [ string ] # which branches the schedule applies to
    exclude: [ string ] # which branches to exclude from the schedule
  always: boolean # whether to always run the pipeline or only if there have been source code changes since the last successful scheduled run. The default is false.
```
```yml
schedules:
# The first schedule, Daily midnight build, runs a pipeline at midnight every day, but only if the code has changed since the last successful scheduled run, for main and all releases/* branches, except the branches under releases/ancient/*.
- cron: '0 0 * * *'
  displayName: Daily midnight build
  branches:
    include:
    - main
    - releases/*
    exclude:
    - releases/ancient/*

# The second schedule, Weekly Sunday build, runs a pipeline at noon on Sundays, whether the code has changed or not since the last run, for all releases/* branches.
- cron: '0 12 * * 0'
  displayName: Weekly Sunday build
  branches:
    include:
    - releases/*
  always: true # Running even when there are no code changes
```

  * Notice:<br>
    * Scheduled triggers defined using the pipeline settings UI take precedence over YAML scheduled triggers.<br>
    * [Cron syntax](https://learn.microsoft.com/en-us/azure/devops/pipelines/process/scheduled-triggers?view=azure-devops&tabs=yaml#cron-syntax)<br>


* [Event-based triggers]()



### [Opting](https://learn.microsoft.com/en-us/azure/devops/pipelines/repos/azure-repos-git?view=azure-devops&tabs=yaml#opting-out-of-ci)
* Disabling the CI triger<br>
```yml
# A pipeline with no CI trigger
trigger: none
```

* Skipping<br>
  * [skip ci] or [ci skip]<br>
  * skip-checks: true or skip-checks:true<br>
  * [skip azurepipelines] or [azurepipelines skip]<br>
  * [skip azpipelines] or [azpipelines skip]<br>
  * [skip azp] or [azp skip]<br>
  * ***NO_CI***<br>

* [Trigger type in conditions](https://learn.microsoft.com/en-us/azure/devops/pipelines/repos/azure-repos-git?view=azure-devops&tabs=yaml#using-the-trigger-type-in-conditions)
```yml
condition: and(succeeded(), ne(variables['Build.Reason'], 'PullRequest'))
```


* Notice:<br>
  * All trigger paths are case-sensitive.<br>
  * You cannot use variables in triggers, as variables are evaluated at runtime (after the trigger has fired).<br>
  * If you use templates to author YAML files, then you can only specify triggers in the main YAML file for the pipeline. You cannot specify triggers in the template files.<br>
  * You can't trigger a pipeline with only a path filter; you must also have a branch filter, and the changed files that match the path filter must be from a branch that matches the branch filter.<br>