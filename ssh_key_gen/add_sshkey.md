# [create ssh-key demo](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
## **Linux command**<br>
* **create an ssh key**<br>
	* **ed25519 key**<br>
	**`ssh-keygen -t ed25519 -C "your_email@example.com"`**<br>
	* **rsa key**<br>
	**`ssh-keygen -t rsa -b 4096 -C "your_email@example.com"`**<br>
* **adding ssh key to the ssh-agent**<br>
	* **start ssh-agent in background**<br>
	**`eval "$(ssh-agent -s)"`
	* **add ssh private key to the ssh-agent**<br>
	**`ssh-add ${the_filepath_of_private_key}"


