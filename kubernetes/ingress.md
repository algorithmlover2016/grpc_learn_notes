# [CLusterIP, NodePort, LoadBalancer Ingress](https://www.ibm.com/cloud/blog/kubernetes-ingress)
## **ClusterIp**
* For internal application access within a Kubernetes cluster, ClusterIP is the preferred method. It is a default setting in Kubernetes and uses an internal IP address to access the service.<br>

## **[Ingress](https://youtu.be/NPFbYpb0I7w)**
* Kubernetes Ingress is an API object that provides routing rules to manage external users' access to the services in a Kubernetes cluster, typically via HTTPS/HTTP.<br>
* Ingress is made up of an Ingress API object and the Ingress Controller.<br>
* Kubernetes Ingress is an API object that describes the desired state for exposing services to the outside of the Kubernetes cluster.
* An Ingress Controller is essential because it is the actual implementation of the Ingress API.
* An Ingress Controller reads and processes the Ingress Resource information and usually runs as pods within the Kubernetes cluster.

### **Ingress Provides**
* Externally reachable URLs for applications deployed in Kubernetes clusters.<br>
* Name-based virtual host and URI-based routing support.<br>
* Load balancing rules and traffic, as well as SSL termination.<br>


