## [Assign Pods to specific Nodes](https://kubernetes.io/docs/tasks/configure-pod-container/assign-pods-nodes/#add-a-label-to-a-node)<br>
```sh
# list labels on nodes
kubectl get nodes --show-labels

# add labels on nodes
kubectl label nodes <your-node-name> disktype=ssd

# verify pods running nodes
kubectl get pods --output=wide
```

```yml
# create a pods that gets scheduled to specific label on nodes
apiVersion: v1
kind: Pod
metadata:
  name: nginx
  labels:
    env: test
spec:
  containers:
  - name: nginx
    image: nginx
    imagePullPolicy: IfNotPresent
  nodeSelector:
    disktype: ssd # specific node label
```
```yml
apiVersion: v1
kind: Pod
metadata:
  name: nginx
spec:
  nodeName: foo-node # schedule pod to specific node
  containers:
  - name: nginx
    image: nginx
    imagePullPolicy: IfNotPresent
```