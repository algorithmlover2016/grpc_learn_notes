# [remove .local suffix from ubuntu hostname](https://askubuntu.com/questions/1026099/how-to-remove-local-suffix-from-ubuntu-hostname)
```sh
echo -e "Host *\n    CanonicalDomains local\n    CanonicalizeHostname yes" >> /etc/ssh_config or ~/.ssh/config
```
# [ssh without secret by public_key](https://segmentfault.com/a/1190000023074072)
* generate id_rsa key pair.<br>
* configure the config file.<br>
* copy public key into remote machine.<br>
    * `ssh-copy-id -i ~/.ssh/id_rsa.pub  username@ip`<br>
* confirm the file in remote machine<br>
    * `chmod 664 ~/.ssh/authorized_keys`<br>