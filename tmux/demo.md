## [tmux](https://github.com/tmux/tmux/wiki/Getting-Started)


### [install](https://github.com/tmux/tmux/wiki/Installing)

### [tutorial]()
#### [basic_use](https://www.ruanyifeng.com/blog/2019/10/tmux.html)<br>
* basic_command<br>
```sh
# install
# Ubuntu 或 Debian
sudo apt-get install tmux

# CentOS 或 Fedora
sudo yum install tmux

# Mac
brew install tmux


# start
tmux

# exit
exit
ctrl + d

# new session
tmux new -s <session-name>

# detch
tmux detach
ctrl + b + d

# list session
tmux ls
tmux list-session

# attach
tmux attach -t <session-name>
tmux attach -t sessioin_id

# kill session
tmux kill-session -t 0
tmux switch -t <session-name>

# rename session
tmux rename-session -t 0 <new-name>

```