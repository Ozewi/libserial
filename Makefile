#
# Makefile de la librer�a serial
# �ltima modificaci�n: 2016.04.01
#

MODULE = libserial
TARGET = libserial.so libserial.a
H_INSTALL_FILES = libserial.h

libserial_dep = libserial.o version.o

CPPFLAGS := $(CPPFLAGS) -fPIC

include ../Makefile.common
