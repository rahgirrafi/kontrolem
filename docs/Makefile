# Minimal Sphinx makefile: `make html` from this directory.
SPHINXBUILD ?= sphinx-build
SOURCEDIR    = .
BUILDDIR     = _build

html:
	$(SPHINXBUILD) -b html "$(SOURCEDIR)" "$(BUILDDIR)/html"

clean:
	rm -rf "$(BUILDDIR)"

.PHONY: html clean
