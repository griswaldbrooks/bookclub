# bookclub
Sandbox for testing implementations from [http://modernrobotics.org/](http://modernrobotics.org/) in rust.

## Build
Build a new development image
```shell
export UIDGID=$(id -u):$(id -g); docker compose -f compose.dev.yml build
```
Start an interactive development container
```shell
export UIDGID=$(id -u):$(id -g); docker compose -f compose.dev.yml run development
```
Build the repository in the container
```shell
username@bookclub-dev:~/ws/bookclub$ cargo build
```

## Run
```shell
username@bookclub-dev:~/ws/bookclub$ cargo build
```

## Lint
```shell
username@bookclub-dev:~/ws/bookclub$ cargo fmt
```

## References
- [PDF of Book 2019](http://hades.mech.northwestern.edu/images/2/25/MR-v2.pdf)
- [Accompanying Videos](https://modernrobotics.northwestern.edu/nu-gm-book-resource/)
- [Development container documentation](https://github.com/griswaldbrooks/development-container/blob/main/README.md)
