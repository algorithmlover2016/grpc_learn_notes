# reference to https://kubernetes.io/docs/concepts/configuration/configmap/
A ConfigMap is an API object used to store non-confidential data in key-value pairs.
Pods can consume ConfigMaps as environment variables, command-line arguments, or as configuration files in a volume.

A ConfigMap allows you to decouple environment-specific configuration from your container images, so that your applications are easily portable.

A ConfigMap is not designed to hold large chunks of data. The data stored in a ConfigMap cannot exceed 1 MiB.
If you need to store settings that are larger than this limit, you may want to consider mounting a volume or use a separate database or file service.

a ConfigMap has "data" and "binaryData" fields. These fields accept key-value pairs as their values.
    Both the data field and the binaryData are optional.
    The data field is designed to contain UTF-8 byte sequences
    while the binaryData field is designed to contain binary data as base64-encoded strings.

    The name of a ConfigMap must be a valid DNS subdomain name(https://kubernetes.io/docs/concepts/overview/working-with-objects/names#dns-subdomain-names).
    Each key under the data or the binaryData field must consist of alphanumeric characters, -, _ or ..
    The keys stored in data must not overlap with the keys in the binaryData field.
    Starting from v1.19, you can add an immutable field to a ConfigMap definition to create an immutable ConfigMap.

ConfigMap demo:
```
apiVersion: v1
kind: ConfigMap
metadata:
  name: game-demo
data:
  # property-like keys; each key maps to a simple value
  player_initial_lives: "3"
  ui_properties_file_name: "user-interface.properties"

  # file-like keys
  game.properties: |
    enemy.types=aliens,monsters
    player.maximum-lives=5
  user-interface.properties: |
    color.good=purple
    color.bad=yellow
    allow.textmode=true
```

There are four different ways that you can use a ConfigMap to configure a container inside a Pod:
    Inside a container command and args
    Environment variables for a container
    Add a file in read-only volume, for the application to read
    Write code to run inside the Pod that uses the Kubernetes API to read a ConfigMap
These different methods lend themselves to different ways of modeling the data being consumed.
    For the first three methods, the kubelet uses the data from the ConfigMap when it launches container(s) for a Pod.
    The fourth method means you have to write code to read the ConfigMap and its data.
    However, because you're using the Kubernetes API directly, your application can subscribe to get updates whenever the ConfigMap changes,
        and react when that happens. By accessing the Kubernetes API directly, this technique also lets you access a ConfigMap in a different namespace.

an example Pod:
```
apiVersion: v1
kind: Pod
metadata:
  name: configmap-demo-pod
spec:
  containers:
    - name: demo
      image: alpine
      command: ["sleep", "3600"]
      env:
        # Define the environment variable
        - name: PLAYER_INITIAL_LIVES # Notice that the case is different here
                                     # from the key name in the ConfigMap.
          valueFrom:
            configMapKeyRef:
              name: game-demo           # The ConfigMap this value comes from.
              key: player_initial_lives # The key to fetch.
        - name: UI_PROPERTIES_FILE_NAME
          valueFrom:
            configMapKeyRef:
              name: game-demo
              key: ui_properties_file_name
      volumeMounts:
      - name: config
        mountPath: "/config"
        readOnly: true
  volumes:
    # You set volumes at the Pod level, then mount them into containers inside that Pod
    - name: config
      configMap:
        # Provide the name of the ConfigMap you want to mount.
        name: game-demo
        # An array of keys from the ConfigMap to create as files
        items:
        - key: "game.properties"
          path: "game.properties"
        - key: "user-interface.properties"
          path: "user-interface.properties"
```
Notes:
    A ConfigMap doesn't differentiate between single line property values and multi-line file-like values. What matters is how Pods and other objects consume those values.
    For this example, defining a volume and mounting it inside the demo container as /config creates two files, /config/game.properties and /config/user-interface.properties,
        even though there are four keys in the ConfigMap. This is because the Pod definition specifies an items array in the volumes section.
        If you omit the items array entirely, every key in the ConfigMap becomes a file with the same name as the key, and you get 4 files.

Using ConfigMaps as files from a Pod
To consume a ConfigMap in a volume in a Pod:
   Create a ConfigMap or use an existing one. Multiple Pods can reference the same ConfigMap.
   Modify your Pod definition to add a volume under .spec.volumes[]. Name the volume anything, and have a .spec.volumes[].configMap.name field set to reference your ConfigMap object.
   Add a .spec.containers[].volumeMounts[] to each container that needs the ConfigMap.
       Specify .spec.containers[].volumeMounts[].readOnly = true and .spec.containers[].volumeMounts[].mountPath to an unused directory name where you would like the ConfigMap to appear.
   Modify your image or command line so that the program looks for files in that directory. Each key in the ConfigMap data map becomes the filename under mountPath.
an example of a Pod that mounts a ConfigMap in a volume:
```
apiVersion: v1
kind: Pod
metadata:
  name: mypod
spec:
  containers:
  - name: mypod
    image: redis
    volumeMounts:
    - name: foo
      mountPath: "/etc/foo"
      readOnly: true
  volumes:
  - name: foo
    configMap:
      name: myconfigmap
```
Notes:
    Each ConfigMap you want to use needs to be referred to in .spec.volumes.
    If there are multiple containers in the Pod, then each container needs its own volumeMounts block, but only one .spec.volumes is needed per ConfigMap.


Mounted ConfigMaps are updated automatically
    When a ConfigMap currently consumed in a volume is updated, projected keys are eventually updated as well.
        The kubelet checks whether the mounted ConfigMap is fresh on every periodic sync. However,
        the kubelet uses its local cache for getting the current value of the ConfigMap. The type of the cache is configurable using the ConfigMapAndSecretChangeDetectionStrategy field
        in the KubeletConfiguration struct. A ConfigMap can be either propagated by watch (default), ttl-based, or by redirecting all requests directly to the API server.
        As a result, the total delay from the moment when the ConfigMap is updated to the moment when new keys are projected to the Pod can be as long as
            the kubelet sync period + cache propagation delay,
            where the cache propagation delay depends on the chosen cache type (it equals to watch propagation delay, ttl of cache, or zero correspondingly).
    ConfigMaps consumed as environment variables are not updated automatically and require a pod restart.

Immutable ConfigMaps:
```
apiVersion: v1
kind: ConfigMap
metadata:
  ...
data:
  ...
immutable: true
```
