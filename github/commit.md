# **[git commit]()**
## [revoke commit](https://www.cnblogs.com/lfxiao/p/9378763.html)<br>
* just revoke your commit  and reserve your codes changes<br>
`git reset --soft HEAD^`
* revoke the commit and the `add` operation, but reserve your changes in workspace<br>
`git reset --mixed --soft HEAD^`
* revoke the commit and `add` operation and clean your workspace<br>
`git reset --hard --soft HEAD^`