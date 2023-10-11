##
##	This file is part of qpOASES.
##
##	qpOASES -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2007-2011 by Hans Joachim Ferreau, Andreas Potschka,
##	Christian Kirches et al. All rights reserved.
##
##	qpOASES is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qpOASES is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qpOASES; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  src/make_linux
##	Author:    Hans Joachim Ferreau
##	Version:   3.0beta
##	Date:      2009-2011
##


##
##	definitions for compiling with gcc under linux
##

CPP = g++
AR  = ar
RM  = rm

OBJEXT = o
LIBEXT = a
EXE =
DEF_TARGET = -o $@

CPPFLAGS = -Wall -pedantic -Wshadow -O3 -finline-functions -DLINUX -D__NO_COPYRIGHT__
#-g -D__DEBUG__ -D__NO_COPYRIGHT__ -D__SUPPRESSANYOUTPUT__

QPOASES_LIB         =  -L${SRCDIR} -lqpOASES
QPOASES_EXTRAS_LIB  =  -L${SRCDIR} -lqpOASESextras

## system BLAS or qpOASES replacement BLAS
#LIB_BLAS = /usr/lib/libblas.so
LIB_BLAS = ${SRCDIR}/BLASReplacement.o

## system LAPACK or qpOASES replacement LAPACK
#LIB_LAPACK = /usr/lib/liblapack.so
LIB_LAPACK = ${SRCDIR}/LAPACKReplacement.o


##
##	end of file
##
