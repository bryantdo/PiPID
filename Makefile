PROG_NAME = temperature
VERSION = 0.1
PYTHON_SCRIPTS = temperature.py
OTHER_FILES = README Makefile 
DIST_FILES = $(PYTHON_SCRIPTS) $(OTHER_FILES)
DIST_DIR = $(PROG_NAME)-$(VERSION)
DIST_NAME = $(PROG_NAME)-$(VERSION).tar.gz

PYTHON_DIR = $(HOME)/.python

GENERATED_FILES = $(DIST_NAME)

all : # all scripts, nothing to compile

clean :
	rm -f $(GENERATED_FILES)

install : uninstall
	cp $(PYTHON_SCRIPTS) $(PYTHON_DIR)

uninstall :
	rm -f $(PYTHON_SCRIPTS:%=$(PYTHON_DIR)/%)

dist :
	mkdir $(DIST_DIR)
	cp $(DIST_FILES) $(DIST_DIR)
	tar -chozf $(DIST_NAME) $(DIST_DIR)
	rm -rf $(DIST_DIR)

check :
	python ./temperature.py
