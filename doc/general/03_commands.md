# ⌨️ Available commands

## General commands

### `b5 run`

Starts the Project (docker-compose up).

### `b5 halt`

Stops the Project (docker-compose down).

### `b5 shell`

Makes it possible to get a shell of any docker container contained in the project.

Possible arguments:

| `argument`  | `description`          | `optional` | `default` |
|-------------|------------------------|------------|-----------|
| `container` | Container name         | True       | flake8    |
| `command`   | Command to be executed | True       |           |

Usage: `b5 shell <container> <command>`

#### Examples

```shell
# Execute flake8 lint in `components`-folder:
b5 shell flake8 components

# Get Shell in perception container (hypothetic example)
b5 shell perception
```

## Project setup / maintenance

### `b5 install`

Setup the project. Has to be run after cloning the project.

### `b5 update`

Update the project.

## Project linting

### `b5 lint`

Runs the project linters. More documentation about linting can be found [here](./linting.md).

### `b5 python:lint`

Runs the python linter. More documentation about linting can be found [here](./linting.md).

### `b5 markdown:lint`

Runs the markdown linter. More documentation about linting can be found [here](./linting.md).
