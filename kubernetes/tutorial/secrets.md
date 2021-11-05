# reference to https://kubernetes.io/docs/concepts/configuration/secret/
To use a Secret, a Pod needs to reference the Secret. A Secret can be used with a Pod in three ways:
    As files in a volume mounted on one or more of its containers.
    As container environment variable.
    By the kubelet when pulling images for the Pod.

The name of a Secret object must be a valid DNS subdomain name. You can specify the "data" and/or the "stringData" field
    when creating a configuration file for a Secret.
The "data" and the "stringData" fields are optional.
The values for all keys in the "data" field have to be base64-encoded strings. If the conversion to base64 string is not desirable,
    you can choose to specify the stringData field instead, which accepts arbitrary strings as values.

The keys of "data" and "stringData" must consist of alphanumeric characters, -, _ or ..
All key-value pairs in the "stringData" field are internally merged into the "data" field.
If a key appears in both the "data" and the "stringData" field, the value specified in the "stringData" field takes precedence

Types of Secret:
```
    Builtin Type	                    Usage
    Opaque	                            arbitrary user-defined data
    kubernetes.io/service-account-token	service account token
    kubernetes.io/dockercfg	            serialized ~/.dockercfg file
    kubernetes.io/dockerconfigjson	    serialized ~/.docker/config.json file
    kubernetes.io/basic-auth	        credentials for basic authentication
    kubernetes.io/ssh-auth	            credentials for SSH authentication
    kubernetes.io/tls	                data for a TLS client or server
    bootstrap.kubernetes.io/token	    bootstrap token data
```
You can define and use your own Secret type by assigning a non-empty string as the type value for a Secret object.
An empty string is treated as an Opaque type.
Kubernetes doesn't impose any constraints on the type name. However, if you are using one of the builtin types, you must meet all the requirements defined for that type.

    Opaque secrets:
        When you create a Secret using kubectl, you will use the generic subcommand to indicate an Opaque Secret type.
```
        # creates an empty Secret of type Opaque
        kubectl create secret generic empty-secret
```
    Service account token Secrets:
        When using this Secret type, you need to ensure that the kubernetes.io/service-account.name annotation is set to an existing service account name.
        A Kubernetes controller fills in some other fields such as the kubernetes.io/service-account.uid annotation and the token key in the data field set to actual token content.

```
        # declares a service account token Secret:
        apiVersion: v1
        kind: Secret
        metadata:
          name: secret-sa-sample
          annotations:
            kubernetes.io/service-account.name: "sa-name"
        type: kubernetes.io/service-account-token
        data:
          # You can include additional key value pairs as you do with Opaque Secrets
          extra: YmFyCg==
```
    Docker config Secrets:
        You can use one of the following type values to create a Secret to store the credentials for accessing a Docker registry for images.
```
        # The kubernetes.io/dockercfg type is reserved to store a serialized ~/.dockercfg which is the legacy format for configuring Docker command line.
        # When using this Secret type, you have to ensure the Secret data field contains a .dockercfg key whose value is content of a ~/.dockercfg file encoded in the base64 format.
        kubernetes.io/dockercfg

        # The kubernetes.io/dockerconfigjson type is designed for storing a serialized JSON that follows the same format rules as the ~/.docker/config.json file
        # which is a new format for ~/.dockercfg.
        # When using this Secret type, the data field of the Secret object must contain a .dockerconfigjson key,
        # in which the content for the ~/.docker/config.json file is provided as a base64 encoded string.
        kubernetes.io/dockerconfigjson

```
        Note: If you do not want to perform the base64 encoding, you can choose to use the stringData field instead.

        When you create these types of Secrets using a manifest,
        the API server checks whether the expected key does exists in the data field,
        and it verifies if the value provided can be parsed as a valid JSON.
        The API server doesn't validate if the JSON actually is a Docker config file.

        ```
        # use kubectl to create a Docker registry Secret, you can do:
        kubectl create secret docker-registry secret-tiger-docker \
          --docker-username=tiger \
          --docker-password=pass113 \
          --docker-email=tiger@acme.com \
          --docker-server=my-registry.example:5000
        ```
    Basic authentication Secret:
        When using this Secret type, the data field of the Secret must contain the following two keys:
```
        username: the user name for authentication;
        password: the password or token for authentication.

        Note:Both values for the above two keys are base64 encoded strings.
        You can, of course, provide the clear text content using the stringData for Secret creation.
```

        ```
        # an example config for a basic authentication Secret:
        apiVersion: v1
        kind: Secret
        metadata:
          name: secret-basic-auth
        type: kubernetes.io/basic-auth
        stringData:
          username: admin
          password: t0p-Secret

        ```
    SSH authentication secrets:
        When using this Secret type, you will have to specify a ssh-privatekey key-value pair in the data (or stringData) field as the SSH credential to use.
```
        # an example config for a SSH authentication Secret:
        apiVersion: v1
        kind: Secret
        metadata:
          name: secret-ssh-auth
        type: kubernetes.io/ssh-auth
        data:
          # the data is abbreviated in this example
          ssh-privatekey: |
                  MIIEpQIBAAKCAQEAulqb/Y ...
```
        Caution: SSH private keys do not establish trusted communication between an SSH client and host server on their own.
                A secondary means of establishing trust is needed to mitigate "man in the middle" attacks, such as a known_hosts file added to a ConfigMap.

    TLS secrets:
        When using this type of Secret, the tls.key and the tls.crt key must be provided in the data (or stringData) field of the Secret configuration,
            although the API server doesn't actually validate the values for each key.
        ```
        # an example config for a TLS Secret
        apiVersion: v1
        kind: Secret
        metadata:
          name: secret-tls
        type: kubernetes.io/tls
        data:
          # the data is abbreviated in this example
          tls.crt: |
                MIIC2DCCAcCgAwIBAgIBATANBgkqh ...
          tls.key: |
                MIIEpgIBAAKCAQEA7yn3bRHQ5FHMQ ...
        ```
        When creating a TLS Secret using kubectl, you can use the tls subcommand as shown in the following example:
```
        kubectl create secret tls my-tls-secret \
          --cert=path/to/cert/file \
          --key=path/to/key/file
        # The public/private key pair must exist beforehand.
        # The public key certificate for --cert must be .PEM encoded (Base64-encoded DER format), and match the given private key for --key.
        # The private key must be in what is commonly called PEM private key format, unencrypted.
        # In both cases, the initial and the last lines from PEM (for example, --------BEGIN CERTIFICATE----- and -------END CERTIFICATE---- for a certificate) are not included.
```
    Bootstrap token Secrets:
        A bootstrap token Secret is usually created in the kube-system namespace and named in the form bootstrap-token-<token-id> where <token-id> is a 6 character string of the token ID.
        ```
        # As a Kubernetes manifest, a bootstrap token Secret might look like the following:
        apiVersion: v1
        kind: Secret
        metadata:
          name: bootstrap-token-5emitj
          namespace: kube-system
        type: bootstrap.kubernetes.io/token
        data:
          auth-extra-groups: c3lzdGVtOmJvb3RzdHJhcHBlcnM6a3ViZWFkbTpkZWZhdWx0LW5vZGUtdG9rZW4=
          expiration: MjAyMC0wOS0xM1QwNDozOToxMFo=
          token-id: NWVtaXRq
          token-secret: a3E0Z2lodnN6emduMXAwcg==
          usage-bootstrap-authentication: dHJ1ZQ==
          usage-bootstrap-signing: dHJ1ZQ==
        ```

        A bootstrap type Secret has the following keys specified under data:
        ```
        token-id: A random 6 character string as the token identifier. Required.
        token-secret: A random 16 character string as the actual token secret. Required.
        description: A human-readable string that describes what the token is used for. Optional.
        expiration: An absolute UTC time using RFC3339 specifying when the token should be expired. Optional.
        usage-bootstrap-<usage>: A boolean flag indicating additional usage for the bootstrap token.
        auth-extra-groups: A comma-separated list of group names that will be authenticated as in addition to the system:bootstrappers group.
        ```

        The above YAML may look confusing because the values are all in base64 encoded strings. In fact, you can create an identical Secret using the following YAML:
        ```
        apiVersion: v1
        kind: Secret
        metadata:
          # Note how the Secret is named
          name: bootstrap-token-5emitj
          # A bootstrap token Secret usually resides in the kube-system namespace
          namespace: kube-system
        type: bootstrap.kubernetes.io/token
        stringData:
          auth-extra-groups: "system:bootstrappers:kubeadm:default-node-token"
          expiration: "2020-09-13T04:39:10Z"
          # This token ID is used in the name
          token-id: "5emitj"
          token-secret: "kq4gihvszzgn1p0r"
          # This token can be used for authentication
          usage-bootstrap-authentication: "true"
          # and it can be used for signing
          usage-bootstrap-signing: "true"
        ```

## Managing Secrets using kubectl (reference to https://kubernetes.io/docs/tasks/configmap-secret/managing-secret-using-kubectl/)
    Create a Secret:
        ```
        # a database connection string consists of a username and password.
        # You can store the username in a file ./username.txt and the password in a file ./password.txt on your local machine.

        # In these commands, the -n flag ensures that the generated files do not have an extra newline character at the end of the text.
        # This is important because when kubectl reads a file and encodes the content into a base64 string, the extra newline character gets encoded too.
        echo -n 'admin' > ./username.txt
        echo -n '1f2d1e2e67df' > ./password.txt


        # packages these files into a Secret and creates the object on the API server.
        kubectl create secret generic db-user-pass \
            --from-file=./username.txt \
            --from-file=./password.txt

        # The default key name is the filename. You can optionally set the key name using --from-file=[key=]source. For example:
        kubectl create secret generic db-user-pass \
          --from-file=username=./username.txt \
          --from-file=password=./password.txt

        # Caution: You do not need to escape special characters in password strings that you include in a file.

        # You can also provide Secret data using the --from-literal=<key>=<value> tag.
        # This tag can be specified more than once to provide multiple key-value pairs.
        # Caution: special characters such as $, \, *, =, and ! will be interpreted by your shell and require escaping.
        
        # In most shells, the easiest way to escape the password is to surround it with single quotes (').
        # For example, if your password is S!B\*d$zDsb=, run the following command:
        kubectl create secret generic db-user-pass \
          --from-literal=username=devuser \
          --from-literal=password='S!B\*d$zDsb='
        ```
    Verify the Secret:
    ```
        # Check that the Secret was created
        kubectl get secrets

        # view a description of the Secret:
        kubectl describe secrets/db-user-pass
    ```
    Decoding the Secret:
    ```
        # To view the contents of the Secret you created, run the following command:
        kubectl get secret db-user-pass -o jsonpath='{.data}'

        # decode the password data, if The output is similar to: {"password":"MWYyZDFlMmU2N2Rm","username":"YWRtaW4="}
        echo 'MWYyZDFlMmU2N2Rm' | base64 --decode
    ```
    Clean Up:
    ```
    kubectl delete secret db-user-pass
    ```

## Managing Secrets using Configuration File (reference to https://kubernetes.io/docs/tasks/configmap-secret/managing-secret-using-config-file/)
    Create the Config file:
        You can create a Secret in a file first, in JSON or YAML format, and then create that object.
        The Secret resource contains two maps:
        ```
            # The data field is used to store arbitrary data, encoded using base64.
            data
            # The stringData field is provided for convenience, and it allows you to provide Secret data as unencoded strings.
            stringData.
        ```
        Notes: The keys of data and stringData must consist of alphanumeric characters, -, _ or ..
        
        ```
        # to store two strings in a Secret using the data field, convert the strings to base64 as follows:
        echo -n 'admin' | base64
        # The output is similar to: YWRtaW4=

        echo -n '1f2d1e2e67df' | base64
        # The output is similar to: MWYyZDFlMmU2N2Rm

        # Write a Secret config file that looks like this:
        apiVersion: v1
        kind: Secret
        metadata:
          name: mysecret
        type: Opaque
        data:
          username: YWRtaW4=
          password: MWYyZDFlMmU2N2Rm

        # Note that the name of a Secret object must be a valid DNS subdomain name.
        ```

        Note:
        ```
            The serialized JSON and YAML values of Secret data are encoded as base64 strings.
            Newlines are not valid within these strings and must be omitted.
            When using the base64 utility on Darwin/macOS, users should avoid using the -b option to split long lines.
            Conversely, Linux users should add the option -w 0(zero) to base64 commands or the pipeline base64 | tr -d '\n' if the -w option is not available.
        ```

        For certain scenarios, you may wish to use the stringData field instead.
        This field allows you to put a non-base64 encoded string directly into the Secret, and the string will be encoded for you when the Secret is created or updated.


        Configuration file demo:
        ```
        # if your application uses the following configuration file:
        apiUrl: "https://my.api.com/api/v1"
        username: "<user>"
        password: "<password>"

        # store this in a Secret using the following definition
        apiVersion: v1
        kind: Secret
        metadata:
          name: mysecret
        type: Opaque
        data:
            username: YWRtaW4=
        stringData:
          config.yaml: |
            apiUrl: "https://my.api.com/api/v1"
            username: <user>
            password: <password>

        # Note: If a field, such as username, is specified in both data and stringData, the value from stringData is used.
        ```
    Create the Secret object:
    ```
        kubectl apply -f ./secret.yaml
    ```

    Check the Secret:
        Note: The stringData field is a write-only convenience field. It is never output when retrieving Secrets.
    ```
        kubectl get secret mysecret -o yaml
    ```

    Clean Up:
    ```
        kubectl delete secret mysecret
    ```

## Managing Secrets using Kustomize (reference to https://kubernetes.io/docs/tasks/configmap-secret/managing-secret-using-kustomize/)
    Create the Kustomization file:
        You can generate a Secret by defining a secretGenerator in a "kustomization.yaml" file by
        ```
            referencing other existing files.
            providing some literals
            providing .env files
        ```

        Demo:
        ```
        # the following kustomization file references the ./username.txt and the ./password.txt files:
        secretGenerator:
        - name: db-user-pass
          files:
          - username.txt
          - password.txt

        # the following kustomization.yaml file contains two literals for username and password respectively
        secretGenerator:
        - name: db-user-pass
          literals:
          - username=admin
          - password=1f2d1e2e67df

        #  the following kustomization.yaml file pulls in data from .env.secret file:
        secretGenerator:
        - name: db-user-pass
          envs:
          - .env.secret
        ```

    Create the Secret:
    ```
    # Apply the directory containing the kustomization.yaml to create the Secret.
    kubectl apply -k .
    ```

    Check the Secret created:
    ```
    # check that the secret was created
    kubectl get secrets
    # if The output is similar to:
    # NAME                             TYPE                                  DATA      AGE
    # db-user-pass-96mffmfh4k          Opaque                                2         51s


    # view a description of the secret
    kubectl describe secrets/db-user-pass-96mffmfh4k
    ```

    Clean Up:
    ```
    kubectl delete secret db-user-pass-96mffmfh4k
    ```
