# template-python

### Setup

- Ensure tools are installed:
  - A version of python3, preferably 3.9
  - [Pre-Commit](https://pre-commit.com/) we can use
    `python3 -m pip install pre-commit` or `brew install pre-commit`
  - [Poetry](https://python-poetry.org) we can use
    `curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python3 -`

### Template Setup
1. Delete `CHANGELOG.md`.
2. Change the vesion number in `pyproject.toml` so the version number is `0.0.0`. Also, delete all existing releases on the repo.
3. Find-and-replace all instances of `template-python` and `template_python` to the name of the new package you are creating. Note that directory / package names must have underscores instead of hyphens (because `-` is a minus sign in python!).
4. Setup branch protection rules and required PR reviews.
5. Delete this 'template setup' section :-)
