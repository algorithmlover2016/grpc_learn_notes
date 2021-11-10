#reference to https://kubernetes.io/docs/tutorials/kubernetes-basics/
# Using Minikube to Create a Cluster (https://kubernetes.io/docs/tutorials/kubernetes-basics/create-cluster/cluster-intro/)

# A Kubernetes cluster consists of two types of resources:
#     The Control Plane coordinates the cluster
#     Nodes are the workers that run applications

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

# Kubernetes Pods https://kubernetes.io/docs/tutorials/kubernetes-basics/explore/explore-intro/
# Kubernetes Nodes: A Node is a worker machine in Kubernetes and may be either a virtual or a physical machine, depending on the cluster
# Every Kubernetes Node runs at least:
#   Kubelet, a process responsible for communication between the Kubernetes control plane and the Node; it manages the Pods and the containers running on a machine.
#   A container runtime (like Docker) responsible for pulling the container image from a registry, unpacking the container, and running the application.

# command:
kubectl get
    kubectl get pods
kubectl describe
    kubectl describe pods

kubectl logs
    # Note: We don’t need to specify the container name, because we only have one container inside the pod.
    kubectl logs $POD_NAME
kubectl exec
    # list the environment variables
    # Again, worth mentioning that the name of the container itself can be omitted since we only have a single container in the Pod.
    kubectl exec $POD_NAME -- env

    #  start a bash session in the Pod’s container
    kubectl exec -ti $POD_NAME -- bash
