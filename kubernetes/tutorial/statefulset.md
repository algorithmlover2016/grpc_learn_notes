# [StatefulSets](https://kubernetes.io/docs/concepts/workloads/controllers/statefulset/)
## **Direcption**<br>
* Manages the deployment and scaling of a set of Pods, and provides guarantees about the ordering and uniqueness of these Pods.<br>
    * Like a Deployment, a StatefulSet manages Pods that are based on an identical container spec.<br>
    * Unlike a Deployment, a StatefulSet maintains a sticky identity for each of their Pods. These pods are created from the same spec, but are not interchangeable<br>
    * Each has a persistent identifier that it maintains across any rescheduling.<br>
* If you want to use storage volumes to provide persistence for your workload, you can use a StatefulSet as part of the solution.<br>
## [Using StatefulSets](https://kubernetes.io/docs/concepts/workloads/controllers/statefulset/#using-statefulsets)
* Stable, unique network identifiers.<br>
* Stable, persistent storage.<br>
* Ordered, graceful deployment and scaling.<br>
* Ordered, automated rolling updates.<br>
* **Caution**:<br>
    *  If an application doesn't require any stable identifiers or ordered deployment, deletion, or scaling,<br>
       you should deploy your application using a workload object that provides a set of stateless replicas.<br>
       [Deployment](https://kubernetes.io/docs/concepts/workloads/controllers/deployment/) or [ReplicaSet](https://kubernetes.io/docs/concepts/workloads/controllers/replicaset/) may be better suited to your stateless needs.<br>
## [Limitations](https://kubernetes.io/docs/concepts/workloads/controllers/statefulset/#limitations)
* The storage for a given Pod must either be provisioned by a PersistentVolume Provisioner based on the requested storage class, or pre-provisioned by an admin.<br>
* Deleting and/or scaling a StatefulSet down will not delete the volumes associated with the StatefulSet.<br>
* StatefulSets currently require a Headless Service to be responsible for the network identity of the Pods. You are responsible for creating this Service.<br>
* StatefulSets do not provide any guarantees on the termination of pods when a StatefulSet is deleted.<br>
    * To achieve ordered and graceful termination of the pods in the StatefulSet, it is possible to scale the StatefulSet down to 0 prior to deletion.<br>
* When using Rolling Updates with the default Pod Management Policy (OrderedReady), it's possible to get into a broken state that requires manual intervention to repair.

## [Components](https://kubernetes.io/docs/concepts/workloads/controllers/statefulset/#components)
```yaml
apiVersion: v1
kind: Service
metadata:
  name: nginx
  labels:
    app: nginx
spec:
  ports:
  - port: 80
    name: web
  clusterIP: None
  selector:
    app: nginx
---
apiVersion: apps/v1
kind: StatefulSet
metadata:
  name: web
spec:
  selector:
    matchLabels:
      app: nginx # has to match .spec.template.metadata.labels
  serviceName: "nginx"
  replicas: 3 # by default is 1
  minReadySeconds: 10 # by default is 0
  template:
    metadata:
      labels:
        app: nginx # has to match .spec.selector.matchLabels
    spec:
      terminationGracePeriodSeconds: 10
      containers:
      - name: nginx
        image: k8s.gcr.io/nginx-slim:0.8
        ports:
        - containerPort: 80
          name: web
        volumeMounts:
        - name: www
          mountPath: /usr/share/nginx/html
  volumeClaimTemplates:
  - metadata:
      name: www
    spec:
      accessModes: [ "ReadWriteOnce" ]
      storageClassName: "my-storage-class" # need user redefined
      resources:
        requests:
          storage: 1Gi
```
```yaml
---
kind: StorageClass
apiVersion: storage.k8s.io/v1
metadata:
  name: premium-storage
provisioner: kubernetes.io/azure-disk
reclaimPolicy: Delete
parameters:
  storageaccounttype: Premium_LRS
  kind: Managed
---
kind: StatefulSet
apiVersion: apps/v1
metadata:
  labels:
    app: statefulset_name
  name: statefulset_name
spec:
  replicas: 1
  selector:
    matchLabels:
      app: statefulset_name # must be .spec.template.metadata.labels.app
  serviceName: statefulset_name-service # better keep consistence with .metadata.name, it's a headless Service, and is used to control the network domain.
  template:
    metadata:
      labels:
        app: statefulset_name
    spec:
      containers:
      - name: container_name
        image: image_address
        volumeMounts:
        - name: persistent-volume
          mountPath: /data
        resources:
          limits:
            nvidia.com/gpu: 1
        env:
        - name: role
          value: prod
        imagePullPolicy: Always
  volumeClaimTemplates:
    - metadata:
        name: persistent-volume
      spec:
        accessModes: ["ReadWriteOnce"]
        storageClassName: premium-storage
        resources:
          requests:
            storage: 128G
```
* **Caution**:<br>
* Must set the **`.spec.selector`** field of a StatefulSet to match the labels of its **`.spec.template.metadata.labels`**<br>
* Can set the **`.spec.volumeClaimTemplates`** which can provide stable storage using **[PersistentVolumes](https://kubernetes.io/docs/concepts/storage/persistent-volumes/)** provisioned by a PersistentVolume Provisioner
* [Pod Identity](https://kubernetes.io/docs/concepts/workloads/controllers/statefulset/#pod-identity)<br>
* [Ordinal Index](https://kubernetes.io/docs/concepts/workloads/controllers/statefulset/#ordinal-index)<br>
* [Stable Network ID](https://kubernetes.io/docs/concepts/workloads/controllers/statefulset/#stable-network-id)<br>
    * pod's hostname: **`$(statefulset name)-$(ordinal)`**<br>
    * domain form: **`$(service name).$(namespace).svc.cluster.local`**, where "cluster.local" is the cluster domain
    * pod dns: **`$(podname).$(governing service domain)`**, where the governing service is defined by the serviceName field on the StatefulSet.
* [Stable Storage](https://kubernetes.io/docs/concepts/workloads/controllers/statefulset/#stable-storage)

## [Statefulset basics](https://kubernetes.io/docs/tutorials/stateful-application/basic-stateful-set/)
### [Creating a StatefulSet](https://kubernetes.io/docs/tutorials/stateful-application/basic-stateful-set/#creating-a-statefulset)
**The example creates a headless Service, `nginx`, to publish the IP addresses of Pods in the StatefulSet, `web`.**<br>
* First, create a headless service, named `nginx`, to publish IP address<br>
* Second, create a statefulset, named `web`.
```yaml
apiVersion: v1
kind: Service
metadata:
  name: nginx
  labels:
    app: nginx
spec:
  ports:
  - port: 80
    name: web
  clusterIP: None
  selector:
    app: nginx
---
apiVersion: apps/v1
kind: StatefulSet
metadata:
  name: web
spec:
  serviceName: "nginx"
  replicas: 2
  selector:
    matchLabels:
      app: nginx
  template:
    metadata:
      labels:
        app: nginx
    spec:
      containers:
      - name: nginx
        image: k8s.gcr.io/nginx-slim:0.8
        ports:
        - containerPort: 80
          name: web
        volumeMounts:
        - name: www
          mountPath: /usr/share/nginx/html
  volumeClaimTemplates:
  - metadata:
      name: www
    spec:
      accessModes: [ "ReadWriteOnce" ]
      resources:
        requests:
          storage: 1Gi
```
### Do some operation
* Watch the creation of the statefulset's pods<br>
`kubectl get pods -w -l app=nginx`
`kubectl get pods -l app=nginx`

* Delete pods in a label<br>
`kubectl delete pod -l app=nginx`

* Login to pod<br>
`kubectl exec -it pod_name -- /bin/bash`

* create the service or statefulset defined in yaml<br>
`kubectl apply -f file_name.yaml`

* Get service in kubernetes<br>
`kubectl get service service_name`

* Get statefulset in kubernetes<br>
`kubectl get statefulset statefulset_name`

* Execute shell command in pods<br>
    * Get hostname<br>
    ```sh
        for i in 0 1;
        do
            kubectl exec "statefulset_name-$i" -- sh -c 'hostname';
        done

        for i in 0 1; # index
        do
            kubectl exec "statefulset_name-$i" -- sh -c 'echo "$(hostname)" > /usr/share/nginx/html/index.html';
        done

        for i in 0 1; # index
        do
            kubectl exec -i -t "statfulset_name-$i" -- curl http://localhost/;
        done
    ```
    * Open a terminal to run sh command<br>
    ```sh
        # create a new shell
        kubectl run -i --tty --image busybox:1.28 dns-test --restart=Never --rm /bin/sh
        # and when entering the new shell terminal, can do sh command
        nslookup pod_name
        # after do your jobs, to exit
        exit
    ```

    * Get container image<br>
    ```sh
        for i in 0 1; # the number of pods in a statefulset
        do
            kubectl get pod $statefulset_name-$i --template '{{range $i, $c := .spec.containers}}{{$c.image}}{{end}}';
            echo;
        done
    ```

* Get PersistentVolumeClaims(PVC)<br>
`kubectl get pvc`
`kubectl get pvc -l app=nginx`


* Scaling a StatefulSet<br>
    * **kubectl scale**<br>
        * scale by parameters<br>
        `kubectl scale sts $statefulset_name --replicas=$new_int_number` # sts is an abbreviation for statefulset
    * **kubectl patch**<br>
        * scale by parameters<br>
        `kubectl patch sts $statefulset_name -p '{"spec":{"replicas":3}}`
    * **How add pods**<br>
        * The StatefulSet controller created each Pod sequentially with respect to its ordinal index, and it waited for each Pod's predecessor to be Running and Ready before launching the subsequent Pod.
    * **Ordered Pod Termination**<br>
        * The controller deleted one Pod at a time, in reverse order with respect to its ordinal index, and it waited for each to be completely shutdown before deleting the next.<br>
    * **Caution**:<br>
        * The PersistentVolumes mounted to the Pods of a StatefulSet are not deleted when the StatefulSet's Pods are deleted.<br>
        * This is still true when Pod deletion is caused by scaling the StatefulSet down<br>

* Updating StatefulSets<br>
    * **strategies**<br>
        * **The strategy used is determined by the `spec.updateStrategy` field of the StatefulSet API**<br>
        * **Valid update strategies**<br>
            * **`RollingUpdate`**<br>
            * **`OnDelete`**<br>
                * Not automatically update Pods when a modification is made to the StatefulSet's `.spec.template` field.
                * This strategy can be selected by setting the `.spec.template.updateStrategy.type` to `OnDelete`<br>
    * **Rolling update**<br>
    The `RollingUpdate` update strategy will update all Pods in a StatefulSet, in reverse ordinal order, while respecting the StatefulSet guarantees.<br>
        * Apply the `RollingUpdate` update strategy<br>
        `kubectl patch statefulset $statefulset_name -p '{"spec":{"updateStrategy":{"type":"RollingUpdate"}}}'`
        * Change container image<br>
        `kubectl patch statefulset $statefulset_name --type='json' -p='[{"op": "replace", "path": "/spec/template/spec/containers/0/image", "value":"gcr.io/google_containers/nginx-slim:0.8"}]'`

* Staging an Update<br>
A staged update will keep all of the Pods in the StatefulSet at the current version while allowing mutations to the StatefulSet's `.spec.template`.<br>
    * The ordinal of the Pod which is less than the `partition` specified by the updateStrategy will keep its original configuration if it is deleted or otherwise terminated.<br>
    * The ordinal of the Pod which is larger than or equal to the `partition` specified by the updateStrategy will apply the change.<br>
    * Set `partition` parameter of the `RollingUpdate` Strategy<br>
    `kubectl patch statefulset $statefulset_name -p '{"spec":{"updateStrategy":{"type":"RollingUpdate","rollingUpdate":{"partition":$int_num}}}}'`
        * By set  `partition` from large number to small number, we can implement Phased Roll Outs.<br>
            * First set to a larger number because updating from end to front.<br>
            `kubectl patch statefulset $statefulset_name -p '{"spec":{"updateStrategy":{"type":"RollingUpdate","rollingUpdate":{"partition":$larger_int_num}}}}'`
            * Second set to a smaller number because updating from end to front.<br>
            `kubectl patch statefulset $statefulset_name -p '{"spec":{"updateStrategy":{"type":"RollingUpdate","rollingUpdate":{"partition":$smaller_int_num}}}}'`
            * Finally set to 0 to update all pods because updating from end to front.<br>
            `kubectl patch statefulset $statefulset_name -p '{"spec":{"updateStrategy":{"type":"RollingUpdate","rollingUpdate":{"partition":0}}}}'`

* Deleting StatefulSets<br>
    * Non-Cascading Delete, the StatefulSet's Pods are not deleted when the StatefulSet is deleted.<br>
    Supply the `--cascade=orphan` parameter to the command. This parameter tells Kubernetes to only delete the StatefulSet, and to not delete any of its Pods.
    `kubectl delete statefulset $statefulset_name --cascade=orphan`
        * As the `$statefulset_name` StatefulSet has been deleted, `$statefulset_name-$ordinal_idx` has not been relaunched if it is delete or terminated.<br>
        ```sh
        kubectl delete statefulset web --cascade=orphan
        kubectl delete pod web-0 # after delete the pod, it will not automatically recreated because its statefulset has been delete.
        kubectl apply -f web.yaml # if we apply the yaml again which means we will create the statefulset again.
        # The pods belonging to the statefulset will be scheduled again, and the deleted or terminated pods will be created again without manually operations.
        ```
    * In a Cascading Delete, both the StatefulSet and its Pods are deleted.<br>
    `kubectl delete statefulset $statefulset_name`
    `kubectl delete -f file_name.yaml`

    * **Caution**:<br>
        * If we delete statefulset by `kubectl detele statefulset $statefulset_name`, we have deleted the statefulset service manually when we need ***clean up***<br>
        `kubectl delete service $service_name_generated_by_statefulset`
        `kubectl delete svc $service_name_generated_by_statefulset`

### [Pod Management Policy](https://kubernetes.io/docs/tutorials/stateful-application/basic-stateful-set/#pod-management-policy)
* **By set parameter `.spec.podManagementPolicy`**<br>
* **`OrderedReady`(Default)**<br>
* **`Parallel**<br>
```yaml
apiVersion: v1
kind: Service
metadata:
  name: nginx
  labels:
    app: nginx
spec:
  ports:
  - port: 80
    name: web
  clusterIP: None
  selector:
    app: nginx
---
apiVersion: apps/v1
kind: StatefulSet
metadata:
  name: web
spec:
  serviceName: "nginx"
  podManagementPolicy: "Parallel"
  replicas: 2
  selector:
    matchLabels:
      app: nginx
  template:
    metadata:
      labels:
        app: nginx
    spec:
      containers:
      - name: nginx
        image: k8s.gcr.io/nginx-slim:0.8
        ports:
        - containerPort: 80
          name: web
        volumeMounts:
        - name: www
          mountPath: /usr/share/nginx/html
  volumeClaimTemplates:
  - metadata:
      name: www
    spec:
      accessModes: [ "ReadWriteOnce" ]
      resources:
        requests:
          storage: 1Gi

```

















