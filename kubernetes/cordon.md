# [cordon](https://kubernetes.io/docs/reference/generated/kubectl/kubectl-commands#cordon)<br>
Mark node as unschedulable.<br>
## Command<br>
*   `kubectl cordon NODE`<br>
    *   `kubectl cordon node-name-0 node-name-1 node-name-2 node-name-3`<br>
    *   `kubectl drain node-name-0 node-name-1 node-name-2 --ignore-daemonsets --delete-emptydir-data`<br>
# [drain](https://kubernetes.io/docs/reference/generated/kubectl/kubectl-commands#drain)<br>
Drain node in preparation for maintenance.<br>
## Command:<br>
*   `kubectl drain NODE`<br>
# [uncordon]()<br>
Mark node as schedulable.<br>
## Command:<br>
*   `kubectl uncordon NODE0 NODE1 NODE2`<br>


