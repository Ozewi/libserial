#
# Makefile de la librería serial
# Última modificación: 2016.03.16
#

MODULE = libserial
TARGET = libserial.so libserial.a
H_INSTALL_FILES = libserial.h

libserial_dep = libserial.o

CPPFLAGS := $(CPPFLAGS) -fPIC

include ../Makefile.common
