# [fork](https://gaohaoyang.github.io/2015/04/12/Syncing-a-fork/)
```sh
# for check
git remote -v
    # the output will be:
        # origin  https://github.com/YOUR_USERNAME/YOUR_FORK.git (fetch)
        # origin  https://github.com/YOUR_USERNAME/YOUR_FORK.git (push)
        # eg:
            # origin  https://github.com/algorithmlover2016/CLIP.git (fetch)
            # origin  https://github.com/algorithmlover2016/CLIP.git (push)

# add upstream
git remote add upstream https://github.com/ORIGINAL_OWNER/ORIGINAL_REPOSITORY.git
    # eg:
        git remote add upstream https://github.com/openai/CLIP.git

# for check again
git remote -v
    # the output will be:
        # origin    https://github.com/YOUR_USERNAME/YOUR_FORK.git (fetch)
        # origin    https://github.com/YOUR_USERNAME/YOUR_FORK.git (push)
        # upstream  https://github.com/ORIGINAL_OWNER/ORIGINAL_REPOSITORY.git (fetch)
        # upstream  https://github.com/ORIGINAL_OWNER/ORIGINAL_REPOSITORY.git (push)
        # eg:
            # origin  https://github.com/algorithmlover2016/CLIP.git (fetch)
            # origin  https://github.com/algorithmlover2016/CLIP.git (push)
            # upstream        https://github.com/openai/CLIP.git (fetch)
            # upstream        https://github.com/openai/CLIP.git (push)

# fetech upstream branch, get upstream/main branch here
git fetch upstream
   # the output will be:
       # remote: Enumerating objects: 17, done.
       # remote: Counting objects: 100% (17/17), done.
       # remote: Compressing objects: 100% (2/2), done.
       # remote: Total 10 (delta 6), reused 10 (delta 6), pack-reused 0
       # Unpacking objects: 100% (10/10), 1.10 KiB | 3.00 KiB/s, done.
       # From https://github.com/openai/CLIP
       #  * [new branch]      main       -> upstream/main

# check branch
git branch -a

# checkout to local branch
git checkout {local_branch_name}
    # eg:
        git checkout main

# merge upstream branch into your local branch
git merge upstream/{branch_name}
    # eg:
        git merge upstream/main

# push to your fork remote
git push
git push origin {branch_name}
    # eg:
        git push origin main
```
```sh
# fork sync example:
git clone https://github.com/algorithmlover2016/segment-anything.git
cd segment-anything
git remote add upstream https://github.com/facebookresearch/segment-anything.git
git fetch upstream
git checkout main
git pull --rebase upstream main # git merge upstream main
git pull
git push origin main
```
