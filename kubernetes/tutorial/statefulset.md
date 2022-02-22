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