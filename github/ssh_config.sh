# reference to https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent
ssh-keygen -t ed25519 -C "yubo.upt@gmail.com"

# cat and paste to github.com ssh keys
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519_docker # not the file with .pub, add the private file
# test
ssh -v -T github.com
chmod 700 ~/.ssh && chmod 600 ~/.ssh/*
echo "Host github.com
    HostName github.com
    PreferredAuthentications publickey
    User algorithmlover2016
    IdentityFile ~/.ssh/id_ed25519_docker
    AddKeysToAgent yes" > ~/.ssh/config

# config user.email and user.name
git config --global user.email "yubo.upt@gmail.com"
git config --global user.name "algorithmlover2016"
