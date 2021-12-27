# [Overview of kubectl](https://kubernetes.io/docs/reference/kubectl/overview/)
* **configuration**<br>
    * **default: `$HOME/.kube`**<br>
    * **config: `$KUBECONFIG`**<br>
    * **flag: `--kubeconfig`**<br>
## Syntax:<br>
* **format**<br>
`kubectl [command] [TYPE] [NAME] [flags]`
    * **command**: Specifies the [operations](https://kubernetes.io/docs/reference/kubectl/overview/#operations)<br>
    * **TYPE**:<br>
        * **Specifies the [resource type](https://kubernetes.io/docs/reference/kubectl/overview/#resource-types)**<br>
        * **Case-insensitive, singular, plural, or abbreviated forms**<br>
    * **NAME**:<br>
        * **Specifies the name of the resource.**<br>
        * **Case-sensitive**<br>
        * **If the NAME is ommited, details for all resources**<br>
        * **Specify multiple resources**<br>
            * **by type and name**<br>
                * **same type and different names**<br>
                `kubectl command TYPE name1 name2 name3 etc`<br>
                * **different type and different name**<br>
                `kubectl command TYPE1/name1 TYPE2/name2 TYPE3/name3 etc`<br>
            * **by files**<br>
            `kubectl command -f file1 file2 etc`<br>
    * **flags** : Specifies optional flags<br>
    * ***help***: `kubectl hellp` for more information<br>

## In-cluster authentication
* **First determine if running within a pod and thus in the cluster by checking the followings:**<br>
    * **Environment variables**<br>
        * ***`KUBERNETES_SERVICE_HOST`***<br>
        * ***`KUBERNETES_SERVICE_PORT`***<br>
    * **The existence of a service account token file at `/var/run/secrets/kubernetes.io/serviceaccount/token`**<br>
* **If the upper environment variables are set and token file is mounted at specified location, looks up the namespace of that Service Account**<br>
* **If not, kubectl runs outside of a cluster, and if you don't specify a namespace, it will acts against the `default` namespace**<br>

### set pod namespace: POD_NAMESPACE<br>
* **default is unset, so `default` is used when needing namespace**<br>
* **determine if a resource is namespaced**<br>
`kubectl api-resources`<br>
* **overrides the $POD_NAMESPACE value**<br>
`kubectl [command] [TYPE] [NAME] [flags] --namespace namespace_value`<br>
`kubectl [command] [TYPE] [NAME] [flags] -n namespace_value`<br>


