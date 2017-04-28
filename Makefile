# This is a very simple Makefile for compiling and installing
# COIL.
#
# This file might change as my Makefile-writing skills improve,
# but it at least should be enough to get the basic things going
# for people who are happy with the default settings.
#

AR = ar
CC = gcc -g
INSTALL = /usr/local/lib
INCLUDE = /usr/local/include

.PHONY:  default all install uninstall clean docs

default: all

install: src/libcreateoi.a src/createoi.h
	cp $< $(INSTALL)
	cp src/createoi.h $(INCLUDE)
	ranlib $(INSTALL)/libcreateoi.a

all: src/createoi.o
	$(AR) rcs src/libcreateoi.a $<

src/createoi.o: src/createoi.c src/createoi.h
	$(CC) -c -o src/createoi.o $<

uninstall:
	rm $(INSTALL)/libcreateoi.a
	rm $(INCLUDE)/createoi.h

docs:
	@echo Not yet supported.  For now, use your favorite TeX editor to compile \
	manual.tex in the docs/ directory.

clean:
	rm src/libcreateoi.a
	rm src/createoi.o
