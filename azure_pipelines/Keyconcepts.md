## [Key Concepts](https://learn.microsoft.com/en-us/azure/devops/pipelines/get-started/key-pipelines-concepts?view=azure-devops)
* A trigger tells a Pipeline to run.<br>
* A pipeline is made up of one or more stages. A pipeline can deploy to one or more environments.<br>
* A stage is a way of organizing jobs in a pipeline and each stage can have one or more jobs.<br>
* Each job runs on one agent. A job can also be agentless.<br>
* Each agent runs a job that contains one or more steps.<br>
* A step can be a task or script and is the smallest building block of a pipeline.<br>
* A task is a pre-packaged script that performs an action, such as invoking a REST API or publishing a build artifact.<br>
* An artifact is a collection of files or packages published by a run.<br>