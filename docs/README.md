# Documentation

Sphinx sources for <https://rahgirrafi.github.io/kontrolem/>,
built and deployed by [`.github/workflows/docs.yml`](../.github/workflows/docs.yml)
on every push to `main` (GitHub Pages, "GitHub Actions" source).

Build locally:

```bash
pip install -r docs/requirements.txt
make -C docs html          # or: sphinx-build -b html docs docs/_build/html
xdg-open docs/_build/html/index.html
```

No ROS installation is needed to build the docs: the API reference imports
the packages straight from `../packages/` with ROS 2 and Pinocchio mocked
(see `conf.py`).
