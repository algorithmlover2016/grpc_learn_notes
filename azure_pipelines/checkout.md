## [checkout](https://learn.microsoft.com/en-us/azure/devops/pipelines/repos/azure-repos-git?view=azure-devops&tabs=yaml#checkout)
* checkout cmd:<br>
`git -c fetch --force --tags --prune --prune-tags --progress --no-recurse-submodules origin --depth=1`
* [checkout grammar]()<br>
```yml
steps:
- checkout: self  # self represents the repo where the initial Pipelines YAML file was found
  fetchTags: true
  clean: boolean  # whether to fetch clean each time
    # if set true
    # git clean -ffdx
    # git reset --hard HEAD
  fetchDepth: number  # the depth of commits to ask Git to fetch
  lfs: boolean  # whether to download Git-LFS files
  submodules: true | recursive  # set to 'true' for a single level of submodules or 'recursive' to get submodules of submodules
  path: string  # path to check out source code, relative to the agent's build directory (e.g. \_work\1)
  persistCredentials: boolean  # set to 'true' to leave the OAuth token in the Git config after the initial fetch
```

### Notice:<br>
* `git -c http.https://<url of submodule repository>.extraheader="AUTHORIZATION: Basic <BASE64_ENCODED_STRING>" submodule update --init --recursive`
    * `BASE64_ENCODED_STRING=base64.encode("pat:$PAT")`