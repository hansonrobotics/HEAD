#
# Development by Carl J. Nobile
#
include include.mk

PREFIX		= $(shell pwd)
BASE_DIR	= $(shell echo $${PWD\#\#*/})
PACKAGE_DIR	= $(BASE_DIR)-motors-$(VERSION)
DOCS_DIR	= $(PREFIX)/docs
LOGS_DIR	= $(PREFIX)/logs

#----------------------------------------------------------------------
all	: tar

#----------------------------------------------------------------------
tar	: clean api-docs
	@(cd ..; tar -czvf $(PACKAGE_DIR).tar.gz --exclude=".git" \
          --exclude=".gitignore" --exclude=".tox"  $(BASE_DIR))

tests	:
	@(export PYTHONPATH=$(PREFIX); python motors/tests/test_qik2s9v1.py;)

api-docs: clean
	@(cd $(DOCS_DIR); make)

dist	: clean api-docs
	python setup.py sdist

upload	: clobber
	python setup.py sdist upload

#----------------------------------------------------------------------

clean	:
	$(shell cleanDirs.sh clean)
	@rm -rf *.egg-info
	@rm -rf pololu-motors-1.0
	@rm -rf dist build

clobber	: clean
	@rm -f logs/*.log
	@(cd $(DOCS_DIR); make clobber)
