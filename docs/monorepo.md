# Monorepo Information

Information about the MonoRepo

## About

The repository is to serve as a single source of truth for all development and production code that we use in WATonomous. The main goal is to simplify remote development for WATonomous members and to provide a seamless and intuitive environment even for less experienced members. As such, if you have any suggestions or questions about this repository, please bring them up in Discord in #infrastructure-general or message @Charles Zhang (Flare#2992). 

The second goal is to provide a reproducable and lightweight environment that is shared for both development and production. Using this environment code developed with our simulators will reflect the behaviour of the code in the final car. See the Technical Specification FAQ for more details about how and why we have set up this repository as it is.

## Why a monorepo?

First, what is a monorepo? A monolithic software repository (monorepo) is a single software repository that stores all code and assets for all projects. Monorepos are a version control popular strategy employed by compaines such as Google [(See here for a great read about google's monorepo)](https://cacm.acm.org/magazines/2016/7/204032-why-google-stores-billions-of-lines-of-code-in-a-single-repository/fulltext), Facebook, Microsoft, Uber, AirBnB and Twitter [(Wikipedia: Monorepo)](https://en.wikipedia.org/wiki/Monorepo). though they typically don't use Git as their version control system. This might seem strange as many probably havent heard of monorepos. So, why a monorepo?

A monorepo's primary advantage is that each commit hash is a complete source of truth and snapshot of the pipeline. For our autonomous pipeline, dependencies are critical and difficult to manage with many self-contained repositories. Instead of having manually update and maintain the complex denepdency tree in both production and development, we can simply change a single commit hash and be sure that (with automated testing) a single commit hash represents a working pipeline.

Using a monorepo also allows for more collaboration between the teams. The perception team can easily see what the Path Planning team is working on and make modifications or improvements to code outside of their specific domain. This allows for flexible code ownership. Motivating this further, changes to the API of a specific component often require changes to many other modules. Instead of having to synchronize multiple merge requests across multiple repositories, all code review can be done in one place and is visible by all teams. 

Monorepos also make [Continuous Integration](https://en.wikipedia.org/wiki/Continuous_integration) easier. In a nutshell, continuous integration (CI) is the process of regularly and automatically collecting the all code in one place to run automated unit tests, integration tests, compilation tests, and continuous deployment (automatically deploying to users, CI/CD). CI ensures that work done by developers will not diverge over time. CI also ensures a baseline level of code quality through regular testing.  With a monorepo, we have a single place to implement unit testing and CI scripts instead of them being dispersed across multiple repositories. Aggregating code is also significantly easier.

## Why Docker?, What is Docker Compose?

In a nutshell, Docker is a way of creating isolated environments, called "Containers", that are tailored to run a specific application. Containers are similar to a virutal machine, though they are lighter and more secure. These containers are reproducable and isolated from the host machine's environment. As such, we guarantee that code developed by one user on their setup seamlessly translates to a different user on a different setup.

Refer to https://docs.docker.com/get-started/overview/ for more information

Docker Compose is a way of managing a series of containers that together form an application (In this case, our autonomous stack). We can configure our services (sub-components of the application) using a YAML file. We can then start and manage all containers in the application through a convenient commandline tool.

Refer to https://docs.docker.com/compose/ for more information
