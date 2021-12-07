# [kubernetes DaemonSet](https://phoenixnap.com/kb/kubernetes-daemonset#:~:text=A%20DaemonSet%20is%20an%20active%20Kubernetes%20object%20managed,the%20desired%20state%20with%20the%20current%20observed%20state.)
## **Feature**<br>
* **A DaemonSet allows you to overcome Kubernetes’ scheduling limitations and makes sure that a specific app gets deployed on all the nodes within the cluster.**<br>
* **A DaemonSet is an active Kubernetes object managed by a controller. You can declare your desired state, indicating that a specific Pod needs to be present on every node.**<br>
* **The reconciliation control loop is going to compare the desired state with the current observed state.**<br>
    * **If an observed node does not have a matching Pod, the DaemonSet controller is going to create one automatically.**<br>
* **The Pods created by DaemonSet controllers are ignored by the Kubernetes scheduler and exist as long as the node itself.**<br>
* **A DaemonSet creates a Pod on every node by default.**<br>
    * **If necessary, you can limit the number of acceptable nodes by using a node selector. The DaemonSet controller is going to create Pods only on nodes that match the predefined `nodeSelector` field in the YAML file.**<br>

## **exceptionally well suited for long-running services**<br>
* **Logs collection**<br>
* **Node resource monitoring (frameworks such as Prometheus)**<br>
* **Cluster storage**<br>
* **Infrastructure-related Pods (system operations)**<br>

## **Create a DaemonSet**<br>
* The `daemonset-node-exporter.yaml` file below deploys a ***`Prometheus`*** `node-exporter`, within the `monitoring` namespace, to monitor hardware usage metrics on every node in the cluster.<br>
```
apiVersion: apps/v1
kind: DaemonSet
metadata:
  name: node-exporter
  namespace: monitoring
  labels:
    name: node-exporter
spec:
  template:
    metadata:
      labels:
        name: node-exporter
      annotations:
         prometheus.io/scrape: "true"
         prometheus.io/port: "9100"
    spec:
      hostPID: true
      hostIPC: true
      hostNetwork: true
      containers:
        - ports:
            - containerPort: 9100
              protocol: TCP
          resources:
            requests:
              cpu: 0.15
          securityContext:
            privileged: true
          image: prom/node-exporter:v0.15.2
          args:
            - --path.procfs
            - /host/proc
            - --path.sysfs
            - /host/sys
            - --collector.filesystem.ignored-mount-points
            - '"^/(sys|proc|dev|host|etc)($|/)"'
          name: node-exporter
          volumeMounts:
            - name: dev
              mountPath: /host/dev
            - name: proc
              mountPath: /host/proc
            - name: sys
              mountPath: /host/sys
            - name: rootfs
              mountPath: /rootfs
      volumes:
        - name: proc
          hostPath:
            path: /proc
        - name: dev
          hostPath:
            path: /dev
        - name: sys
          hostPath:
            path: /sys
        - name: rootfs
          hostPath:
            path: /
```
* **apply the `daemonset-node-exporter.yaml` file**<br>
```
kubectl apply -f daemonset-node-exporter.yaml
```
* **confirm state with `describe` command**<br>
```
kubectl describe daemonset node-exporter -n monitoring
```
* **confirm by listing all pods**<br>
```
kubectl get pod -o wide -n monitoring
```
* **get configuration file (`.yaml`) from pod or daemonset**<br>
```
kubectl get daemonsets node-exporter -n monitoring -o yaml
kubectl get pods node-exporter-suffix -n monitoring -o yaml
```
* **Limit Daemonset by node-seclector**<br>
    * **Node selectors are part of the `nodeSelector` field within the DaemonSet YAML file.**<br>
        * **In the following example, a DaemonSet is going to deploy Nginx only on nodes labeled as `ssd=true`.**<br>
        **label a node named node-1 `ssd=true`**<br>
        ```
        kubectl label nodes node01 ssd=true
        ```
        ```
        apiVersion: apps/v1 
        kind: "DaemonSet" 
        metadata: 
            labels: 
                app: nginx 
                ssd: "true" 
            name: nginx-ssd-storage 
        spec: 
            template: 
                metadata: 
                    labels: 
                        app: nginx
                        ssd: "true" 
                spec: 
                    nodeSelector: 
                        ssd: "true" 
                    containers:
                        - name: nginx 
                          image: nginx:1.10.0
        ```


* **`kubectl rollout` command to check the status of a DaemonSet rolling upgrade**<br>
```
kubectl rollout status ds/daemonset-node-exporter -n monitoring
```

* **Delete a Daemonset**<br>
```
kubectl delete -f daemonset-node-exporter.yaml -n monitoring
```