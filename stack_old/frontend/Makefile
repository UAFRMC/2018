COMPILER=g++

DIRS=-I/usr/local/include -L/usr/local/lib -I../include

SOIL_DIR=../include/SOIL
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

CFLAGS=-O -Wall -Wno-deprecated-declarations -Wno-char-subscripts

all: frontend

frontend: main.cpp $(SOIL)
	$(COMPILER) $^ $(LIB) $(CFLAGS) $(DIRS) -o $@

frontend_msl: main.cpp $(SOIL)
	$(COMPILER) $^ $(LIB) $(CFLAGS) $(DIRS) -o $@ -D MSL -std=c++11

clean:
	rm -f frontend frontend.exe frontend_msl frontend_msl.exe
