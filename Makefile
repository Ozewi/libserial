#
# Makefile de la librer�a serial
# �ltima modificaci�n: 23.04.2014
#

MODULE = libserial
TARGET = libserial.so libserial.a
H_INSTALL_FILES = libserial.h

libserial_dep = libserial.o

include ../Makefile.common
