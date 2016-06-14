#
# Makefile de la librería libserial
# Última modificación: 2016.05.27
#

MODULE = libserial
TARGET = libserial.so libserial.a
H_INSTALL_FILES = libserial.h

libserial_dep = libserial.o version.o

TESTS = test_libserial
test_libserial_dep = test_libserial.o serial Utility

CPPFLAGS := $(CPPFLAGS) -fPIC

include ../Makefile.common
