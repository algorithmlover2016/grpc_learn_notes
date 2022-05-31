[get command](https://kubernetes.io/zh/docs/reference/kubectl/_print/)
```sh
# get pod special field, refer to https://stackoverflow.com/questions/58297188/how-to-access-key-in-a-map-returned-by-kubectl
kubectl get node  -o custom-columns=Num:'{.status.capacity.nvidia\.com/gpu}',Name:'{.metadata.name}'
# get one node's gpu number
kubectl get node node_name -o=jsonpath='{.status.capacity.nvidia\.com/gpu}'
# get each number of every node's gpu in the cluster
kubectl get node  -o=jsonpath='{.items[*].status.capacity.nvidia\.com/gpu}'
# get the raw message
kubectl get node  -o=jsonpath='{@}'

kubectl get node  -o yaml
kubectl get node  -o json
kubectl get node  -o wide
kubectl get node  -o name
```