/**
 Repeatedly dump the data in the marker.bin file.
*/
#include <fstream>
#include <iostream>
#include <unistd.h> /* for sleep */
#include "location_binary.h"

int main(int argc,char *argv[]) {
	const char *path="marker.bin";
	bool always=false;
	if (argc>1) path=argv[1];
	if (argc>2) always=true;
	location_reader reader;
	while (true) {
		location_binary bin;
		if (reader.updated(path,bin)) {
			if (always || bin.valid) 
			printf("Marker %d: Camera %.3f %.3f %.3f meters, heading %.1f degrees, %d vidcap\n",
				bin.marker_ID, bin.x,bin.y,bin.z,bin.angle,bin.vidcap_count
			);
			fflush(stdout);
		}
		usleep(20*1000);
	}
}

