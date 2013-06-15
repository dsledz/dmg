# Sample Makefile for OS X

CC= clang++
GTEST_DIR= gtest-1.6.0
SDL_DIR= ../SDL
CPPFLAGS=  -I${GTEST_DIR}/include -g -Wall -Werror -std=c++11 -stdlib=libc++ -DGTEST_USE_OWN_TR1_TUPLE=1 -I${SDL_DIR}/include ${CFLAGS} ${FLAGS}
GTEST_LIB= libgtest.a
SDL_LIBS= -L${SDL_DIR}/lib -lSDLmain -lSDL -framework OpenGL -framework Cocoa

CHECK_SRC= cpu_test.cc
CPU_OBJS= cpu.o
DMG_OBJS= main.o graphics.o control.o

all: ${CPU_OBJS} check dmg

${GTEST_LIB}:
	${CC} ${CPPFLAGS} -I${GTEST_DIR} -c ${GTEST_DIR}/src/gtest-all.cc
	ar -rv ${GTEST_LIB} gtest-all.o

cpu.o: cpu.cc cpu.h
	${CC} ${CPPFLAGS} -c cpu.cc

graphics.o: graphics.cc graphics.h
	${CC} ${CPPFLAGS} -c graphics.cc

control.o: control.cc control.h
	${CC} ${CPPFLAGS} -c control.cc

main.o: cpu.h main.cc
	${CC} ${CPPFLAGS} -c main.cc

check: ${GTEST_LIB} ${CPU_OBJS} ${CHECK_SRC}
	${CC} ${CPPFLAGS} -I${GTEST_DIR} -c ${GTEST_DIR}/src/gtest_main.cc
	${CC} ${CPPFLAGS} ${GTEST_LIB} ${CHECK_SRC} ${CPU_OBJS} gtest_main.o -o check

dmg: ${CPU_OBJS} ${DMG_OBJS}
	${CC} ${CPPFLAGS} ${SDL_LIBS} ${CPU_OBJS} ${DMG_OBJS} -o dmg

clean:
	rm -f *.o *.a check dmg
