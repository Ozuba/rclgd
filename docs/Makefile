# Makefile for Sphinx documentation

SPHINXOPTS      ?=
SPHINXBUILD     ?= sphinx-build
SPHINXSOURCEDIR ?= .
SPHINXBUILDDIR  ?= _build
SPHINXPAPER     ?=
FILELIST        ?=
PROJECT_PATH    ?= ../tests

# User-friendly check for sphinx-build
ifneq ($(shell which $(SPHINXBUILD) >/dev/null 2>&1; echo $$?), 0)
define ERROR_MESSAGE
The '$(SPHINXBUILD)' command was not found!
Make sure you have Sphinx installed, then set the SPHINXBUILD make variable to the full path of the '$(SPHINXBUILD)' executable.
Alternatively you can add the executable's directory to your PATH.
If you don't have Sphinx installed, grab it from http://sphinx-doc.org/
endef
$(error ${ERROR_MESSAGE})
endif

PAPEROPT_a4         = -D latex_paper_size=a4
PAPEROPT_letter     = -D latex_paper_size=letter
COMMONSPHINXOPTS    = $(PAPEROPT_$(SPHINXPAPER)) $(SPHINXOPTS) '$(SPHINXSOURCEDIR)'
DEFAULTSPHINXOPTS   = -d $(SPHINXBUILDDIR)/doctrees $(COMMONSPHINXOPTS)

.PHONY: help clean rst extract

help:
	@$(SPHINXBUILD) -M help $(DEFAULTSPHINXOPTS) $(O) "$(SPHINXBUILDDIR)"

clean:
	@$(SPHINXBUILD) -M clean $(DEFAULTSPHINXOPTS) $(O) "$(SPHINXBUILDDIR)"/clean $(FILELIST)
	rm -rf classes/

extract:
	@echo "Extracting GDExtension XML docs..."
	ros2 run rclgd godot -- --path $(PROJECT_PATH) --headless --doctool ../docs  --gdextension-docs

rst: extract
	@echo "Converting XML to RST..."
	mkdir -p classes
	python3 make_rst.py --output classes/ doc_classes/

# This is used by https://github.com/godotengine/godot-docs-l10n
# See https://github.com/godotengine/godot-docs-l10n/blob/f157c0cacc8a6e542e06e96b983b27de91637f8b/update.sh#L92
gettext:
	$(SPHINXBUILD) -b gettext -t i18n $(COMMONSPHINXOPTS) ../sphinx/templates $(FILELIST)

%:
	$(SPHINXBUILD) -M $@ $(DEFAULTSPHINXOPTS) $(O) "$(SPHINXBUILDDIR)"/$@ $(FILELIST)