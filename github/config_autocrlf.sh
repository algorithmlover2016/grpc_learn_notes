# reference to http://kuanghy.github.io/2017/03/19/git-lf-or-crlf
git config --global core.autocrlf false
git config --global core.safecrlf true

# .gitattributes reference to
# https://docs.github.com/cn/get-started/getting-started-with-git/configuring-git-to-handle-line-endings
# Set the default behavior, in case people don't have core.autocrlf set.
* text=auto

# Explicitly declare text files you want to always be normalized and converted
# to native line endings on checkout.
*.c text=lf
*.h text=lf
*.sh text=lf
*.py text=lf

# Declare files that will always have CRLF line endings on checkout.
*.sln text eol=crlf

# Denote all files that are truly binary and should not be modified.
*.png binary
*.jpg binary
