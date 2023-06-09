## [Trigger Authentication](https://keda.sh/docs/2.10/concepts/authentication/)
```YML
apiVersion: keda.sh/v1alpha1
kind: TriggerAuthentication
metadata:
  name: {trigger-authentication-name}
  namespace: default # must be same namespace as the ScaledObject
spec:
  podIdentity:
      provider: none | azure | azure-workload | aws-eks | aws-kiam | gcp  # Optional. Default: none
      identityId: <identity-id>                                           # Optional. Only used by azure & azure-workload providers.
  secretTargetRef:                                                        # Optional.
  - parameter: {scaledObject-parameter-name}                              # Required.
    name: {secret-name}                                                   # Required.
    key: {secret-key-name}                                                # Required.
  env:                                                                    # Optional.
  - parameter: {scaledObject-parameter-name}                              # Required.
    name: {env-name}                                                      # Required.
    containerName: {container-name}                                       # Optional. Default: scaleTargetRef.envSourceContainerName of ScaledObject
  hashiCorpVault:                                                         # Optional.
    address: {hashicorp-vault-address}                                    # Required.
    namespace: {hashicorp-vault-namespace}                                # Optional. Default is root namespace. Useful for Vault Enterprise
    authentication: token | kubernetes                                    # Required.
    role: {hashicorp-vault-role}                                          # Optional.
    mount: {hashicorp-vault-mount}                                        # Optional.
    credential:                                                           # Optional.
      token: {hashicorp-vault-token}                                      # Optional.
      serviceAccount: {path-to-service-account-file}                      # Optional.
    secrets:                                                              # Required.
    - parameter: {scaledObject-parameter-name}                            # Required.
      key: {hasicorp-vault-secret-key-name}                               # Required.
      path: {hasicorp-vault-secret-path}                                  # Required.
  azureKeyVault:                                                          # Optional.
    vaultUri: {key-vault-address}                                         # Required.
    podIdentity:                                                          # Optional. Required when using pod identity.
      provider: azure | azure-workload                                    # Required.
      identityId: <identity-id>                                           # Optional
    vaultURI: {key-vault-address}                                         # Required.
    credentials:                                                          # Optional. Required when not using pod identity.
      clientId: {azure-ad-client-id}                                      # Required.
      clientSecret:                                                       # Required.
        valueFrom:                                                        # Required.
          secretKeyRef:                                                   # Required.
            name: {k8s-secret-with-azure-ad-secret}                       # Required.
            key: {key-within-the-secret}                                  # Required.
      tenantId: {azure-ad-tenant-id}                                      # Required.
    cloud:                                                                # Optional.
      type: AzurePublicCloud | AzureUSGovernmentCloud | AzureChinaCloud | AzureGermanCloud | Private # Required.
      keyVaultResourceURL: {key-vault-resource-url-for-cloud}             # Required when type = Private.
      activeDirectoryEndpoint: {active-directory-endpoint-for-cloud}      # Required when type = Private.
    secrets:                                                              # Required.
    - parameter: {param-name-used-for-auth}                               # Required.
      name: {key-vault-secret-name}                                       # Required.
      version: {key-vault-secret-version}                                 # Optional.
    - parameter: connection                                               # required
      name: {}
      version: {}
```
### [Authentication Parameters](https://keda.sh/docs/2.10/scalers/azure-storage-queue/#authentication-parameters)
You can authenticate by using pod identity or connection string authentication.
* Connection String Authentication:
    * connection - Connection string for Azure Storage Account.
* Pod identity based authentication:
    * Azure AD Pod Identity or Azure AD Workload Identity providers can be used.
```yml
# used to call TriggerAuthentication when in azure-storage-queue
apiVersion: keda.sh/v1alpha1
kind: TriggerAuthentication
metadata:
  name: azure-queue-auth
spec:
  podIdentity:
    provider: azure | azure-workload
---
apiVersion: keda.sh/v1alpha1
kind: ScaledObject
metadata:
  name: azure-queue-scaledobject
  namespace: default
spec:
  scaleTargetRef:
    name: azurequeue-function
  triggers:
  - type: azure-queue
    metadata:
      # Required
      queueName: functionsqueue
      # Optional, required when pod identity is used
      accountName: storage-account-name
      # Optional: connection OR authenticationRef that defines connection
      connectionFromEnv: STORAGE_CONNECTIONSTRING_ENV_NAME # Default: AzureWebJobsStorage. Reference to a connection string in deployment
      # or authenticationRef as defined below
      #
      # Optional
      queueLength: "5" # default 5
      cloud: Private
      endpointSuffix: queue.local.azurestack.external # Required when cloud=Private
    authenticationRef:
        name: azure-queue-auth # authenticationRef would need either podIdentity or define a connection parameter
```