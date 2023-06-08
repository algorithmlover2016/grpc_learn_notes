```yml
apiVersion: keda.sh/v1alpha1
kind: ScaledObject
metadata:
  name: my-statefulset-scaler
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: StatefulSet
    name: my-statefulset
  pollingInterval: 30
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
```
```sh
kubectl create secret generic my-auth-secret --from-literal=storage-account-name=<your-storage-account-name> --from-literal=storage-account-key=<your-storage-account-key>
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