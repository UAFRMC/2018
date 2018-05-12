OPTS=-O
COMPILER=g++ -std=c++14

DIRS=-I/usr/local/include -L/usr/local/lib -I../include

SOIL_DIR=../../autonomy/include/SOIL
SOIL=$(SOIL_DIR)/stb_image_aug.c $(SOIL_DIR)/SOIL.c

ifeq ($(OS),Windows_NT)
	LIBS+=-lWs2_32
	#ADD LIBS FOR GL/GLUT on windows...
else
	LIB+=-lpthread

	ifeq ($(shell uname),Darwin)
		LIB+=-framework OpenGL -framework GLUT -framework IOKit -framework CoreFoundation
	else
		LIB+=-lGL -lGLU -lglut
	endif
endif

CFLAGS=$(OPTS) -Wall -Wno-deprecated-declarations -Wno-char-subscripts

all: gridnav

gridnav: main.cpp $(SOIL)
	$(COMPILER) $^ $(LIB) $(CFLAGS) $(DIRS) -o $@

clean:
	rm -f gridnav gridnav.exe 
