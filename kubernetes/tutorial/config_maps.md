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

# create configMap
# reference to https://kubernetes.io/docs/tasks/configure-pod-container/configure-pod-configmap/
create ConfigMap command format:
    kubectl create configmap <map-name> <data-source>

Create ConfigMaps from directories:
    When you are creating a ConfigMap based on a directory, kubectl identifies files whose basename is a valid key in the directory and
    packages each of those files into the new ConfigMap. Any directory entries except regular files are ignored (e.g. subdirectories, symlinks, devices, pipes, etc).
```
    # Create the local directory
    mkdir -p configure-pod-container/configmap/
    
    # Download the sample files into `configure-pod-container/configmap/` directory
    wget https://kubernetes.io/examples/configmap/game.properties -O configure-pod-container/configmap/game.properties --no-check-certificate
    wget https://kubernetes.io/examples/configmap/ui.properties -O configure-pod-container/configmap/ui.properties --no-check-certificate
    
    # Create the configmap
    kubectl create configmap game-config --from-file=configure-pod-container/configmap/

    #  display details of the ConfigMap
    kubectl describe configmaps game-config

    # get configmaps 
    kubectl get configmaps game-config -o yaml
```

Create ConfigMaps from files:
```
    # Create the local directory
    mkdir -p configure-pod-container/configmap/
    
    # Download the sample files into `configure-pod-container/configmap/` directory
    wget https://kubernetes.io/examples/configmap/game.properties -O configure-pod-container/configmap/game.properties --no-check-certificate
    wget https://kubernetes.io/examples/configmap/ui.properties -O configure-pod-container/configmap/ui.properties --no-check-certificate
    
    # Create the configmap
    kubectl create configmap game-config-2 --from-file=configure-pod-container/configmap/game.properties

    # display detail of the ConfigMap
    kubectl describe configmaps game-config-2

    # pass in the --from-file argument multiple times to create a ConfigMap from multiple data sources
    kubectl create configmap game-config-3 --from-file=configure-pod-container/configmap/game.properties --from-file=configure-pod-container/configmap/ui.properties

    # display details of the game-config-3e ConfigMap
    kubectl describe configmaps game-config-3

```

BinaryData:
    When kubectl creates a ConfigMap from inputs that are not ASCII or UTF-8, the tool puts these into the binaryData field of the ConfigMap,
        and not in data. Both text and binary data sources can be combined in one ConfigMap.
        If you want to view the binaryData keys (and their values) in a ConfigMap, you can run "kubectl get configmap -o jsonpath='{.binaryData}' <name>."

    Use the option --from-env-file to create a ConfigMap from an env-file:
        Caution: When passing --from-env-file multiple times to create a ConfigMap from multiple data sources, only the last env-file is used.
```
    # Env-files contain a list of environment variables.
    # These syntax rules apply:
    #   Each line in an env file has to be in VAR=VAL format.
    #   Lines beginning with # (i.e. comments) are ignored.
    #   Blank lines are ignored.
    #   There is no special handling of quotation marks (i.e. they will be part of the ConfigMap value)).
    
    # Download the sample files into `configure-pod-container/configmap/` directory
    wget https://kubernetes.io/examples/configmap/game-env-file.properties -O configure-pod-container/configmap/game-env-file.properties --no-check-certificate
    
    # The env-file `game-env-file.properties` looks like below
    cat configure-pod-container/configmap/game-env-file.properties
    enemies=aliens
    lives=3
    allowed="true"
    
    # This comment and the empty line above it are ignored

    # create evn ConfigMap
    kubectl create configmap game-config-env-file \
       --from-env-file=configure-pod-container/configmap/game-env-file.properties

    # get configMap with the format of yaml
    kubectl get configmap game-config-env-file -o yaml
```

```
    # --from-env-file multiple times demo
    # Download the sample files into `configure-pod-container/configmap/` directory
    wget https://kubernetes.io/examples/configmap/ui-env-file.properties -O configure-pod-container/configmap/ui-env-file.properties --no-check-certificate

    # Create the configmap
    kubectl create configmap config-multi-env-files \
        --from-env-file=configure-pod-container/configmap/game-env-file.properties \
        --from-env-file=configure-pod-container/configmap/ui-env-file.properties

    # get the configmap
    kubectl get configmap config-multi-env-files -o yaml
```
Define the key to use when creating a ConfigMap from a file:
    You can define a key other than the file name to use in the data section of your ConfigMap when using the --from-file argument

    kubectl create configmap game-config-4 --from-file=<my-key-name>=<path-to-file>

```
    kubectl create configmap game-config-4 --from-file=game-special-key=configure-pod-container/configmap/game.properties

    # get the configMap
    kubectl get configmaps game-config-4 -o yaml
```

Create ConfigMaps from literal values:
    You can use kubectl create configmap with the --from-literal argument to define a literal value from the command line:

```
    # create configMap by literal
    kubectl create configmap special-config --from-literal=special.how=very --from-literal=special.type=charm

    # get the configMap
    kubectl get configmaps special-config -o yaml
```

Create a ConfigMap from generator:
    kubectl supports kustomization.yaml since 1.14. You can also create a ConfigMap from generators and then apply it to create the object on the Apiserver.
    The generators should be specified in a kustomization.yaml inside a directory.

```
    # Generate ConfigMaps from files:
    # Create a kustomization.yaml file with ConfigMapGenerator
    cat <<EOF >./kustomization.yaml
    configMapGenerator:
    - name: game-config-5
      files:
      - configure-pod-container/configmap/game.properties
    EOF

    # Apply the kustomization directory to create the ConfigMap object
    kubectl apply -k .

    # get all the configMaps
    kubectl get configmap

    # get describitation of specified configMap
    gameConfig5Specified="$(kubectl get configmap | grep 'game-config-5' | awk '{print $1}')"
    kubectl describe configmaps/"${gameConfig5Specified}"

    Note that the generated ConfigMap name has a suffix appended by hashing the contents. This ensures that a new ConfigMap is generated each time the content is modified

```
Define the key to use when generating a ConfigMap from a file:

    # to generate a ConfigMap from files configure-pod-container/configmap/game.properties with the key game-special-key
```
    # Create a kustomization.yaml file with ConfigMapGenerator
    cat <<EOF >./kustomization.yaml
    configMapGenerator:
    - name: game-config-5
      files:
      - game-special-key=configure-pod-container/configmap/game.properties
    EOF

    # Apply the kustomization directory to create the ConfigMap object
    kubectl apply -k .

    # get all the configMaps
    kubectl get configmap

    # get describitation of specified configMap
    gameConfig5Specified="$(kubectl get configmap | grep 'game-config-5' | tail -1 | awk '{print $1}')"
    kubectl describe configmaps/"${gameConfig5Specified}"

```

Generate ConfigMaps from Literals:

```
    # Create a kustomization.yaml file with ConfigMapGenerator
    cat <<EOF >./kustomization.yaml
    configMapGenerator:
    - name: special-config-2
      literals:
      - special.how=very
      - special.type=charm
    EOF

    # Apply the kustomization directory to create the ConfigMap object
    kubectl apply -k .

```

Define container environment variables using ConfigMap data:

    Define a container environment variable with data from a single ConfigMap
```
    # Define an environment variable as a key-value pair in a ConfigMap
    kubectl create configmap special-config --from-literal=special.how=very

    # Assign the special.how value defined in the ConfigMap to the SPECIAL_LEVEL_KEY environment variable in the Pod specification.
    cat <<EOF >./pods/pod-single-configmap-env-variable.yaml
    apiVersion: v1
    kind: Pod
    metadata:
      name: dapi-test-pod
    spec:
      containers:
        - name: test-container
          image: k8s.gcr.io/busybox
          command: [ "/bin/sh", "-c", "env" ]
          env:
            # Define the environment variable
            - name: SPECIAL_LEVEL_KEY
              valueFrom:
                configMapKeyRef:
                  # The ConfigMap containing the value you want to assign to SPECIAL_LEVEL_KEY
                  name: special-config
                  # Specify the key associated with the value
                  key: special.how
      restartPolicy: Never
EOF
    # Create the Pod:
    kubectl create -f https://kubernetes.io/examples/pods/pod-single-configmap-env-variable.yaml

```

    Define container environment variables with data from multiple ConfigMaps
```
    # create the ConfigMaps
    cat <<EOF >./configure-pod-container/configmap/configmaps.yaml
    apiVersion: v1
    kind: ConfigMap
    metadata:
      name: special-config
      namespace: default
    data:
      special.how: very
    ---
    apiVersion: v1
    kind: ConfigMap
    metadata:
      name: env-config
      namespace: default
    data:
      log_level: INFO
EOF

    kubectl create -f https://kubernetes.io/examples/configmap/configmaps.yaml

    # Define the environment variables in the Pod specification.
    cat <<EOF >./pods/pod-multiple-configmap-env-variable.yaml
    apiVersion: v1
    kind: Pod
    metadata:
      name: dapi-test-pod
    spec:
      containers:
        - name: test-container
          image: k8s.gcr.io/busybox
          command: [ "/bin/sh", "-c", "env" ]
          env:
            - name: SPECIAL_LEVEL_KEY
              valueFrom:
                configMapKeyRef:
                  name: special-config
                  key: special.how
            - name: LOG_LEVEL
              valueFrom:
                configMapKeyRef:
                  name: env-config
                  key: log_level
      restartPolicy: Never
EOF

    # Create the Pod
    kubectl create -f https://kubernetes.io/examples/pods/pod-multiple-configmap-env-variable.yaml
```

Configure all key-value pairs in a ConfigMap as container environment variables:
```
    # Create a ConfigMap containing multiple key-value pairs
    cat <<EOF ./configure-pod-container/configmap/configmap-multikeys.yaml
    apiVersion: v1
    kind: ConfigMap
    metadata:
      name: special-config
      namespace: default
    data:
      SPECIAL_LEVEL: very
      SPECIAL_TYPE: charm
EOF
    
    # Create the ConfigMap:
    kubectl create -f https://kubernetes.io/examples/configmap/configmap-multikeys.yaml

    # Use envFrom to define all of the ConfigMap's data as container environment variables
    cat <<EOF ./pods/pod-configmap-envFrom.yaml
    apiVersion: v1
    kind: Pod
    metadata:
      name: dapi-test-pod
    spec:
      containers:
        - name: test-container
          image: k8s.gcr.io/busybox
          command: [ "/bin/sh", "-c", "env" ]
          envFrom:
          - configMapRef:
              name: special-config
      restartPolicy: Never
EOF

    # Create the Pod:
    kubectl create -f https://kubernetes.io/examples/pods/pod-configmap-envFrom.yaml
```

    Use ConfigMap-defined environment variables in Pod commands:
```
    cat <<EOF ./pods/pod-configmap-env-var-valueFrom.yaml
    apiVersion: v1
    kind: Pod
    metadata:
      name: dapi-test-pod
    spec:
      containers:
        - name: test-container
          image: k8s.gcr.io/busybox
          command: [ "/bin/echo", "$(SPECIAL_LEVEL_KEY) $(SPECIAL_TYPE_KEY)" ]
          env:
            - name: SPECIAL_LEVEL_KEY
              valueFrom:
                configMapKeyRef:
                  name: special-config
                  key: SPECIAL_LEVEL
            - name: SPECIAL_TYPE_KEY
              valueFrom:
                configMapKeyRef:
                  name: special-config
                  key: SPECIAL_TYPE
      restartPolicy: Never
EOF

    # created by running
    kubectl create -f https://kubernetes.io/examples/pods/pod-configmap-env-var-valueFrom.yaml
```

Add ConfigMap data to a Volume

```
    cat <<EOF >./configure-pod-container/configmap/configmap-multikeys.yaml
    apiVersion: v1
    kind: ConfigMap
    metadata:
      name: special-config
      namespace: default
    data:
      SPECIAL_LEVEL: very
      SPECIAL_TYPE: charm
EOF
    # Create the ConfigMap:
    kubectl create -f https://kubernetes.io/examples/configmap/configmap-multikeys.yaml

    cat <<EOF > ./pods/pod-configmap-volume.yaml
    apiVersion: v1
    kind: Pod
    metadata:
      name: dapi-test-pod
    spec:
      containers:
        - name: test-container
          image: k8s.gcr.io/busybox
          command: [ "/bin/sh", "-c", "ls /etc/config/" ]
          volumeMounts:
          - name: config-volume
            # Caution: If there are some files in the /etc/config/ directory, they will be deleted.
            mountPath: /etc/config
      volumes:
        - name: config-volume
          configMap:
            # Provide the name of the ConfigMap containing the files you want
            # to add to the container
            name: special-config
      restartPolicy: Never
EOF

    # Create the Pod:
    kubectl create -f https://kubernetes.io/examples/pods/pod-configmap-volume.yaml
```

Add ConfigMap data to a specific path in the Volume:

```
    # Use the path field to specify the desired file path for specific ConfigMap items.
    cat <<EOF > ./pods/pod-configmap-volume-specific-key.yaml
    apiVersion: v1
    kind: Pod
    metadata:
      name: dapi-test-pod
    spec:
      containers:
        - name: test-container
          image: k8s.gcr.io/busybox
          command: [ "/bin/sh","-c","cat /etc/config/keys" ]
          volumeMounts:
          - name: config-volume
            mountPath: /etc/config
      volumes:
        - name: config-volume
          configMap:
            name: special-config
            items:
            - key: SPECIAL_LEVEL
              path: keys
      restartPolicy: Never
EOF
```
