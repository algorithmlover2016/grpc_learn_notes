# [remove .local suffix from ubuntu hostname](https://askubuntu.com/questions/1026099/how-to-remove-local-suffix-from-ubuntu-hostname)
```sh
echo -e "Host *\n    CanonicalDomains local\n    CanonicalizeHostname yes" >> /etc/ssh_config or ~/.ssh/config
```
# [ssh without secret by public_key](https://segmentfault.com/a/1190000023074072)
