# Creating Base Images

## What is a base image?

In Docker, base images are the starting points for building new Docker images. They are minimal images that contain only the essential components needed for a specific operating system or application framework. Developers use these base images as a foundation to layer additional software, configurations, and code to create custom images tailored to their needs.

Typically, we specify base images in a Dockerfile in the following way:

``` Dockerfile
FROM <base-image-name> as <stage-name>
.
.
.
```

## How are the wato_monorepo base images created?

All wato_monorepo base images are created using [GitHub Workflows](https://docs.github.com/en/actions/using-workflows/about-workflows). GitHub Workflows are automated procedures defined in a GitHub repository that can be triggered by various GitHub events, such as a push or a pull request, to perform tasks like building, testing, and deploying code.

