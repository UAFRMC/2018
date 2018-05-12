OPTS=-g -O3
ARUCO=/usr/local
OPENCV=/usr/lib/x86_64-linux-gnu
CFLAGS=$(OPTS) -L$(OPENCV) -I$(ARUCO)/include/aruco 
LOPENCV=-lopencv_core -lopencv_calib3d -lopencv_highgui -lopencv_features2d -lopencv_imgproc -lopencv_video -lpthread -lm
LFLAGS=$(OPTS) -L$(ARUCO)/lib -laruco -Wl,-rpath,$(ARUCO)/lib -Wl,-rpath,$(OPENCV)/lib $(LOPENCV)


all: camera

camera: main.cpp
	g++ $< -o $@ $(CFLAGS) $(LFLAGS)

clean:
	- rm camera

cat_marker: cat_marker.cpp
	g++ $(OPTS) $< -o $@ -I.

