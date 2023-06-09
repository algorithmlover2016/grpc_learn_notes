## [keda](https://keda.sh/docs/2.10/concepts/)
```yml
apiVersion: keda.sh/v1alpha1
kind: ScaledObject
metadata:
  name: my-statefulset-scaler
  namespace: my-namespace
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: StatefulSet
    name: my-statefulset
  pollingInterval: 30
  cooldownPeriod: 60
  minReplicaCount: 0
  maxReplicaCount: 10
  triggers:
  - type: azure-storage-queue
    metadata:
      queueName: my-queue
      queueLength: "5"
      accountName: my-storage-account
      cloud: AzurePublicCloud
    authenticationRef:
      name: my-auth-secret
  - type: azure-queue
    metadata:
      queueName: my-queue
      connection: my-storage-connection
      messageCount: "5"
```

```sh
kubectl create secret generic my-auth-secret --from-literal=storage-account-name=<your-storage-account-name> --from-literal=storage-account-key=<your-storage-account-key>
kubectl create secret generic my-storage-connection --from-literal=storage-account-name=<your-storage-account-name> --from-literal=storage-account-key=<your-storage-account-key>
kubectl create secret generic my-keyvault-connection --from-literal=keyvault-url=<your-keyvault-url> --from-literal=client-id=<your-client-id> --from-literal=client-secret=<your-client-secret>
```

```yaml
apiVersion: keda.sh/v1alpha1
kind: TriggerAuthentication
metadata:
  name: my-auth-secret
spec:
  secretTargetRef:
    - parameter: storage-account-name
      name: my-storage-account-name
    - parameter: storage-account-key
      name: my-storage-account-key
```
```yml
apiVersion: keda.sh/v1alpha1
kind: TriggerAuthentication
metadata:
  name: my-trigger-auth
spec:
  secretTargetRef:
    name: my-storage-connection
```
```yml
apiVersion: keda.sh/v1alpha1
kind: TriggerAuthentication
metadata:
  name: my-auth-secret
spec:
  keyVaultAuthRef:
    name: my-keyvault-connection
    secretName: my-secret
    secretVersion: my-secret-version
    secretKey: my-secret-key
```
