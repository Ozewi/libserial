#
# Makefile de la librería libserial
# Última modificación: 2025.02.25
#

MODULE = libserial
TARGET = libserial.so libserial.a
H_INSTALL_FILES = libserial.h

libserial_dep = libserial.o version.o

TESTS = test_libserial nanoconsole
test_libserial_dep = test_libserial.o serial Utility
nanoconsole_dep = nanoconsole.o serial Utility pthread

CPPFLAGS := $(CPPFLAGS) -fPIC

include ../Makefile.common
