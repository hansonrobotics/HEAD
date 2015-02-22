#
# docs/Makefile
#
# Development by Carl J. Nobile
#
# This Makefile generates the Pololu Motor API adocumentation.
#
include ../include.mk

FQDN		= $(shell hostname --fqdn)
DOC_DIR		= $(shell pwd)
API_DIR		= pmcapi
URL		= http://${FQDN}${DOC_DIR}/${API_DIR}/index.html
TARBALL_NAME	= pololu_motor_docs

all	: api-docs
#----------------------------------------------------------------------
api-docs:
	@(cd ..; epydoc -v --config docs/epydoc.conf)

#----------------------------------------------------------------------
tar	:
	@(ln -s ${API_DIR} ${TARBALL_NAME}; \
          tar -chzvf ${TARBALL_NAME}.tar.gz ${TARBALL_NAME})
#----------------------------------------------------------------------
clean	:
	@rm -f *~ \#* .\#*

clobber	: clean
	@rm -rf ${API_DIR} ${TARBALL_NAME} ${TARBALL_NAME}.tar.gz
