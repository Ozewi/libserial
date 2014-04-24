#
# Makefile de la librería serial
# Última modificación: 23.04.2014
#

MODULE = libserial
TARGET = libserial.so libserial.a
H_INSTALL_FILES = libserial.h

libserial_dep = libserial.o

include ../Makefile.common
