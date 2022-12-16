# [Taints and Tolerations](https://kubernetes.io/docs/concepts/scheduling-eviction/taint-and-toleration/)
## [taint](https://kubernetes.io/docs/reference/generated/kubectl/kubectl-commands#taint)
```sh
# no pod will be able to schedule onto node1 unless it has a matching toleration.
kubectl taint nodes node1 key1=value1:NoSchedule
kubectl taint NODE NAME KEY_1=VAL_1:TAINT_EFFECT_1 ... KEY_N=VAL_N:TAINT_EFFECT_N

# remove the taint added by the command above
kubectl taint nodes node1 key1=value1:NoSchedule-

```
```yml
# specify a toleration for a pod in the PodSpec
tolerations:
- key: "key1"
  operator: "Equal"
  value: "value1"
  effect: "NoSchedule"

tolerations:
- key: "key1"
  operator: "Exists"
  effect: "NoSchedule"


# an example of a pod that uses tolerations
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
  tolerations:
  - key: "example-key"
    operator: "Exists"
    effect: "NoSchedule"
```
## Notice:<br>
*   The default value for `operator` is Equal.<br>
*   A toleration "matches" a taint if the keys are the same and the effects are the same.<br>
    *   the `operator` is `Exists` (in which case no value should be specified)<br>
    *   the `operator` is `Equal` and the `value`s are equal.<br>
*   An empty key with operator `Exists` matches all keys, values and effects which means this will tolerate everything.<br>
*   An empty `effect` matches all effects with key key1<br>
*   effect:<br>
    *   NoSchedule<br>
        *   pod must meet all the taints if any<br>
    *   PreferNoSchedule<br>
        *   pod must meet all the taints if any<br>
    *   NoExecute<br>
        *   pod must meet all the taints if any<br>
## Examples:<br>
### Example1:<br>
```sh
kubectl taint nodes node1 key1=value1:NoSchedule
kubectl taint nodes node1 key1=value1:NoExecute
kubectl taint nodes node1 key2=value2:NoSchedule
```
```yml
tolerations:
- key: "key1"
  operator: "Equal"
  value: "value1"
  effect: "NoSchedule"
- key: "key1"
  operator: "Equal"
  value: "value1"
  effect: "NoExecute"
```
*   The pod will not be able to schedule onto the node, because there is no toleration matching the third taint.<br>
*   But it will be able to continue running if it is already running on the node when the taint is added, because the third taint is the only one of the three that is not tolerated by the pod.<br>

### Example2:<br>
```yml
tolerations:
- key: "key1"
  operator: "Equal"
  value: "value1"
  effect: "NoExecute"
  tolerationSeconds: 3600
```
*   If this pod is running and a matching taint is added to the node, then the pod will stay bound to the node for 3600 seconds, and then be evicted.<br>
*   If the taint is removed before that time, the pod will not be evicted.<br>

## Use Cases:<br>
### Dedicated Nodes:<br>
*   Add a taint to those nodes  and then add a corresponding toleration to their pods<br>
    *   The pods with the tolerations will then be allowed to use the tainted (dedicated) nodes as well as any other nodes in the cluster.<br>
        `kubectl taint nodes nodename dedicated=groupName:NoSchedule`<br>
    *  Dedicate the nodes to pods and ensure they only use the dedicated nodes.<br>
        `kubectl label nodes <your-node-name> disktype=ssd`<br>
        ```yml
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
            disktype: ssd
        ```
### Nodes with Special Hardware:<br>
```sh
kubectl taint nodes nodename special=true:NoSchedule
kubectl taint nodes nodename special=true:PreferNoSchedule
```
### Taint based Evictions:<br>