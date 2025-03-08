#
# Makefile de la librería libserial
# Última modificación: 2025.03.08
#

MODULE = libserial
TARGET = libserial.so libserial.a
H_INSTALL_FILES = libserial.h

libserial_dep = libserial.o

TESTS = test_libserial nanoterm
test_libserial_dep = test_libserial.o serial
nanoterm_dep = nanoterm.o serial

CPPFLAGS := $(CPPFLAGS) -fPIC

include ../Makefile.common
