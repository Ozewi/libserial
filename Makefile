#
# Makefile de la librer�a serial
# �ltima modificaci�n: 23.01.2006
#
include ../arch.mak

PROJECT = libserial
TARGET  = libserial.so libserial.a

libserial_dep = serial.o

H_INSTALL_FILES = serial.h
DBG_FLAGS = -fno-exceptions -fno-rtti

include ../common.mak
