# [Application Introspection and Debugging](https://kubernetes.io/docs/tasks/debug-application-cluster/debug-application-introspection/)
## ***Using kubectl describe pod to fetch details about pods***

* **get pods**<br>
***`kubectl get pods`***<br>
***`kubectl get pods -o yaml`***<br>
***`kubectl get pod nginx-deployment-1006230814-6winp -o yaml`***<br>

* **retrieve a lot more information about each of these pods**<br>
***`kubectl describe pod nginx-deployment-1006230814-6winp`***<br>

* **list events**
    * **all events**<br>
***`kubectl get events`***
    * **specify namespace**<br>
***`kubectl get events --namespace=my-namespace`***
    * **see events from all namespaces**
***`kubectl get events --all-namespaces`***

* **look at the status of a node**<br>
***`kubectl get nodes`***<br>
***`kubectl get node kubernetes-node-861h -o yaml`***<br>
    * **describe node**<br>
***`kubectl describe node kubernetes-node-861h`***<br>

## ***[Logs](https://kubernetes.io/docs/concepts/cluster-administration/logging/)***
* **a configuration file for a pod that has two sidecar containers**
```
apiVersion: v1
kind: Pod
metadata:
  name: counter
spec:
  containers:
  - name: count
    image: busybox
    args:
    - /bin/sh
    - -c
    - >
      i=0;
      while true;
      do
        echo "$i: $(date)" >> /var/log/1.log;
        echo "$(date) INFO $i" >> /var/log/2.log;
        i=$((i+1));
        sleep 1;
      done
    volumeMounts:
    - name: varlog
      mountPath: /var/log
  - name: count-log-1
    image: busybox
    args: [/bin/sh, -c, 'tail -n+1 -f /var/log/1.log']
    volumeMounts:
    - name: varlog
      mountPath: /var/log
  - name: count-log-2
    image: busybox
    args: [/bin/sh, -c, 'tail -n+1 -f /var/log/2.log']
    volumeMounts:
    - name: varlog
      mountPath: /var/log
  volumes:
  - name: varlog
    emptyDir: {}
```
* **Here are two configuration files that you can use to implement a sidecar container with a logging agent.**
    * **a ConfigMap to configure fluentd**
```
apiVersion: v1
kind: ConfigMap
metadata:
  name: fluentd-config
data:
  fluentd.conf: |
    <source>
      type tail
      format none
      path /var/log/1.log
      pos_file /var/log/1.log.pos
      tag count.format1
    </source>

    <source>
      type tail
      format none
      path /var/log/2.log
      pos_file /var/log/2.log.pos
      tag count.format2
    </source>

    <match **>
      type google_cloud
    </match>
```
* **describes a pod that has a sidecar container running fluentd. The pod mounts a volume where fluentd can pick up its configuration data.**
```
apiVersion: v1
kind: Pod
metadata:
  name: counter
spec:
  containers:
  - name: count
    image: busybox
    args:
    - /bin/sh
    - -c
    - >
      i=0;
      while true;
      do
        echo "$i: $(date)" >> /var/log/1.log;
        echo "$(date) INFO $i" >> /var/log/2.log;
        i=$((i+1));
        sleep 1;
      done
    volumeMounts:
    - name: varlog
      mountPath: /var/log
  - name: count-agent
    image: k8s.gcr.io/fluentd-gcp:1.30
    env:
    - name: FLUENTD_ARGS
      value: -c /etc/fluentd-config/fluentd.conf
    volumeMounts:
    - name: varlog
      mountPath: /var/log
    - name: config-volume
      mountPath: /etc/fluentd-config
  volumes:
  - name: varlog
    emptyDir: {}
  - name: config-volume
    configMap:
      name: fluentd-config
```
***Notes:***
* If you have an application that writes to a single file, it's recommended to set ***`/dev/stdout`*** as the destination rather than implement the streaming sidecar container approach<br>
* It's recommended to use **`stdout`** and **`stderr`** directly and leave rotation and retention policies to the kubelet.<br>

## **[kubelet config](https://kubernetes.io/docs/reference/config-api/kubelet-config.v1beta1/)**
