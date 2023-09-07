# FAQ
## Every time I make a new branch I need to rebuild all the images from scratch

Whenever you make a new branch, new images need to be compiled. However, Docker will use the `main` images as a cache. You probably don't have the `main` images downloaded on your machine.

Try these steps:
```bash
# Log into the docker registry
$ docker login git.uwaterloo.ca:5050
# Pull the latest develop images
$ TAG=develop ./watod2 --all pull
# Start your containers
$ ./watod2 up
```

## "Invalid reference format" when using watod2

If you get an error such as: `ERROR: no such image: git.uwaterloo.ca:5050/watonomous/wato_monorepo/perception:debug-charles/test: invalid reference format`

This is because your branch name has a slash (`/`) in it. This breaks the image naming system that docker uses. Please change your branch name to use dashes `-` instead.

```bash
git checkout -b <NEW_BRANCH_NAME>
# VERIFY all of your changes are saved in your new branch
git branch -d <OLD_BRANCH_NAME>
```