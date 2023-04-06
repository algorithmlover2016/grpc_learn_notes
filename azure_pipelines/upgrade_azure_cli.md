## [azure devops and cli install](https://learn.microsoft.com/en-us/azure/devops/cli/azure-devops-cli-in-yaml?view=azure-devops)
```yml
variables:
- name: variableGroupId

trigger: none
# - main

# pool:
#   vmImage: `ubuntu-latest`

# Run on multiple Microsoft-hosted agent images
strategy:
  matrix:
    linux22:
      imageName: "ubuntu-22.04"
    macOS11:
      imageName: "macos-11"
    windows2022:
      imageName: "windows-2022"
  maxParallel: 3

pool:
  vmImage: $(imageName)

steps:
    - bash: az --version
      displayName: 'Show Azure CLI version'

    # Specify python version
    - task: UsePythonVersion@0
      inputs:
        versionSpec: '3.x'
        architecture: 'x64'

    # Update pip to latest
    - bash: python -m pip install --upgrade pip
      displayName: 'Upgrade pip'

    # # Update to latest Azure CLI version
    # - bash: pip install --pre azure-cli --extra-index-url https://azurecliprod.blob.core.windows.net/edge
    #   displayName: 'Upgrade Azure CLI'

    # Install Azure DevOps CLI extension only on macOS images
    - bash: az extension add -n azure-devops
      condition: contains(variables.imageName, 'mac')
      displayName: 'Install Azure DevOps extension'

    # Azure DevOps CLI extension call that does not require login or credentials
    # since it configures the local environment
    - bash: az devops configure --defaults organization=$(System.TeamFoundationCollectionUri) project=$(System.TeamProject) --use-git-aliases true
      displayName: 'Set default Azure DevOps organization and project'

    # Call that does require credentials, use the System.AccessToken PAT
    # and assign to AZURE_DEVOPS_EXT_PAT which is known to Azure DevOps CLI extension
    - bash: |
        az pipelines build list
        git pr list
      displayName: 'Show build list and PRs'
      env:
        AZURE_DEVOPS_EXT_PAT: $(System.AccessToken)
    
    # Assign the results of an Azure DevOps CLI call to a variable (https://learn.microsoft.com/en-us/azure/devops/cli/azure-devops-cli-in-yaml?view=azure-devops#assign-the-results-of-an-azure-devops-cli-call-to-a-variable)
    - bash: |
        echo "##vso[task.setvariable variable=variableGroupId]$(az pipelines variable-group list --group-name Fabrikam-2021 --query [].id -o tsv)"
        echo "variableGroupId: "$(variableGroupId)
      env:
        AZURE_DEVOPS_EXT_PAT: $(System.AccessToken)
      displayName: 'Get Fabrikam-2021 variable group id'

    - bash: az pipelines variable-group variable list --group-id $(variableGroupId)
      env:
        AZURE_DEVOPS_EXT_PAT: $(System.AccessToken)
      displayName: 'List variables in Fabrikam-2021 variable group'
```