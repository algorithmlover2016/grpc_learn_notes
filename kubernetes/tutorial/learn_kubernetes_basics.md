# [Learn Kubernetes Basics](https://kubernetes.io/docs/tutorials/kubernetes-basics/)

## [Using Minikube to Create a Cluster](https://kubernetes.io/docs/tutorials/kubernetes-basics/create-cluster/cluster-intro/)
* **A Kubernetes cluster consists of two types of resources:**
    * **The Control Plane coordinates the cluster**
    * **Nodes are the workers that run applications**
```
#check minikube properly installed
minikube version
# start the cluster
minikube start # minikube start --memory=2200mb

# interact with Kubernetes
kubectl version

# cluster details
kubectl cluster-info # kubectl cluster-info dump

# view the nodes in the cluster
kubectl get nodes

# Using kubectl to Create a Deployment
# https://kubernetes.io/docs/tutorials/kubernetes-basics/deploy-app/deploy-intro/
# Deploy our app
# kubectl create deployment deployment_name app_image
kubectl create deployment kubernetes-bootcamp --image=gcr.io/google-samples/kubernetes-bootcamp:v1

# list your deployments
kubectl get deployments

# view our app
# run the proxy
kubectl proxy
# test proxy is right
curl http://localhost:8001/version

# get the Pod name
# Pod: A Pod is a Kubernetes abstraction that represents a group of one or more application containers (such as Docker),
#      and some shared resources for those containers. Those resources include:
#           Shared storage, as Volumes
#           Networking, as a unique cluster IP address
#           Information about how to run each container, such as the container image version or specific ports to use

export POD_NAME=$(kubectl get pods -o go-template --template '{{range .items}}{{.metadata.name}}{{"\n"}}{{end}}')
echo Name of the Pod: $POD_NAME
# access the Pod through the API
curl http://localhost:8001/api/v1/namespaces/default/pods/$POD_NAME/
```

## [Kubernetes Pods](https://kubernetes.io/docs/tutorials/kubernetes-basics/explore/explore-intro/)
* ***Kubernetes Nodes: A Node is a worker machine in Kubernetes and may be either a virtual or a physical machine, depending on the cluster***<br>
* ***Every Kubernetes Node runs at least:***<br>
    * **Kubelet, a process responsible for communication between the Kubernetes control plane and the Node; it manages the Pods and the containers running on a machine.**<br>
    * **A container runtime (like Docker) responsible for pulling the container image from a registry, unpacking the container, and running the application.**<br>
### **[command](https://kubernetes.io/docs/reference/generated/kubectl/kubectl-commands)**
* **kubectl get:**<br>
***`kubectl get pods`***
* **kubectl describe**<br>
***`kubectl describe pods`***
* **kubectl logs:**<br>
    ```
    # Note: We don’t need to specify the container name, because we only have one container inside the pod.
    kubectl logs $POD_NAME
    ```
    * **details**
    ```
    Options: (kubectl logs --help)
        --all-containers=false: Get all containers' logs in the pod(s).
        -c, --container='': Print the logs of this container
        -f, --follow=false: Specify if the logs should be streamed.
            --ignore-errors=false: If watching / following pod logs, allow for any errors that occur to be non-fatal
            --insecure-skip-tls-verify-backend=false: Skip verifying the identity of the kubelet that logs are requested from.
        In theory, an attacker could provide invalid log content back. You might want to use this if your kubelet serving
        certificates have expired.
              --limit-bytes=0: Maximum bytes of logs to return. Defaults to no limit.
              --max-log-requests=5: Specify maximum number of concurrent logs to follow when using by a selector. Defaults to 5.
              --pod-running-timeout=20s: The length of time (like 5s, 2m, or 3h, higher than zero) to wait until at least one
        pod is running
              --prefix=false: Prefix each log line with the log source (pod name and container name)
          -p, --previous=false: If true, print the logs for the previous instance of the container in a pod if it exists.
          -l, --selector='': Selector (label query) to filter on.
              --since=0s: Only return logs newer than a relative duration like 5s, 2m, or 3h. Defaults to all logs. Only one of
        since-time / since may be used.
              --since-time='': Only return logs after a specific date (RFC3339). Defaults to all logs. Only one of since-time /
        since may be used.
              --tail=-1: Lines of recent log file to display. Defaults to -1 with no selector, showing all log lines otherwise
        10, if a selector is provided.
              --timestamps=false: Include timestamps on each line in the log output
        
        Usage:
          kubectl logs [-f] [-p] (POD | TYPE/NAME) [-c CONTAINER] [options]
    ```
* **[kubectl exec](https://www.shellhacks.com/kubectl-exec-shell-login-to-pod-container/#:~:text=A%20kubectl%20exec%20command%20serves%20for%20executing%20commands,interactive%20shell%20session%20using%20the%20kubectl%20exec%20command.)**<br>
```
# list the environment variables
# Again, worth mentioning that the name of the container itself can be omitted since we only have a single container in the Pod.
kubectl exec $POD_NAME -- env

# start a bash session in the Pod’s container
# Get interactive shell to a Pod (if the Pod has multiple containers, you will login to a default one, i.e. the first container specified in the Pod’s config.)
kubectl exec -it $POD_NAME -- /bin/bash
kubectl exec --stdin --tty $POD_NAME -- /bin/bash

# To list all the containers in a Kubernetes Pod
kubectl get pod $POD_NAME -o jsonpath='{.spec.containers[*].name}{"\n"}'

# Login to a particular container in the Pod
kubectl exec -it $POD_NAME -c $container_name -- /bin/bash

# Login to a Pod in another Namespace
kubectl exec -it $POD_NAME -n $namespace -- /bin/bash

# Opening a shell when a Pod has more than one container
kubectl exec -i -t my-pod --container main-app -- /bin/bash
```
