## [Authenticate with Azure DevOps](https://learn.microsoft.com/en-us/azure/devops/cli/azure-devops-cli-in-yaml?view=azure-devops#authenticate-with-azure-devops)
```yml
trigger:
- main

pool:
  vmImage: `ubuntu-latest`

steps:
- bash: az --version
  displayName: 'Show Azure CLI version'

- bash: az devops configure --defaults organization=$(System.TeamFoundationCollectionUri) project=$(System.TeamProject) --use-git-aliases true
  displayName: 'Set default Azure DevOps organization and project'

- bash: |
    az pipelines build list
    git pr list
  displayName: 'Show build list and PRs'
  env:
    AZURE_DEVOPS_EXT_PAT: $(System.AccessToken)
```