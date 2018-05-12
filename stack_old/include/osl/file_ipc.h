/**
Trivially simple file-based Inter-Process Communication (IPC)

Only designed to work with fixed-size POD structs that can be stored
as binary data formats on the same machine.

However, either the publisher or subscriber can crash without causing problems
for the other side.

Dr. Orion Lawlor, lawlor@alaska.edu, 2017-05-16 (Public domain)
*/
#ifndef __OSL_FILE_IPC_H
#define __OSL_FILE_IPC_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

#ifndef FILE_IPC_DIRECTORY 

#ifdef _WIN32
#  define FILE_IPC_DIRECTORY "..\\ipc\\"
#else
#  define FILE_IPC_DIRECTORY "/tmp/ipc/"
#endif

#endif

template <class IPC_DATA>
class file_ipc_link {
public:
	// The name of the file (including path) storing our IPC data
	std::string filename;

  struct POD_counter_data {
    long count; // monotonic counter, always counts upward
    long size;  // byte count (and simple sanity check, prevent version mismatch)
    IPC_DATA data;
    long count2; // should equal count (more sanity checking, prevent slicing)
  };
  
  POD_counter_data current;
	POD_counter_data last;
	
	file_ipc_link(const std::string &name) {
	  current.count=-1;
	  last.count=-1;
	  
		filename=FILE_IPC_DIRECTORY + name;
	}
	
	// Publish this updated data to the file:
	void publish(const IPC_DATA &data) {
		FILE *f=fopen(filename.c_str(),"rb+");
		if (!f) {
	    // Make the directory for the file
		  int err=system("mkdir -p " FILE_IPC_DIRECTORY);
		  if (err!=0) fprintf(stderr,"ERROR CREATING IPC DIRECTORY %s\n", FILE_IPC_DIRECTORY);

      // Now make the file itself		
			f=fopen(filename.c_str(),"wb"); 
			if (!f) {
			  fprintf(stderr,"ERROR CREATING IPC FILE %s\n", filename.c_str());
			  exit(1);
			}
		}
		if (f) {
		  current.count++;
		  current.size=sizeof(data);
		  current.data=data;
		  current.count2=current.count;
			fwrite(&current,sizeof(POD_counter_data),1,f);
			fclose(f);
		}
	}
	
	// Check for updated data in the file.  
	//  Returns true if the data has been updated, false if not.
	bool subscribe(IPC_DATA &data) {
		FILE *f=fopen(filename.c_str(),"rb"); // try read first
		bool changed=false;
		if (f) {
			if (fread(&current,sizeof(POD_counter_data),1,f)==1)
			{
			  if (current.count!=last.count) {
			    if (current.size!=sizeof(data)) {
			      fprintf(stderr,"ERROR> IPC FILE %s SIZE MISMATCH: expected %ld, got %ld\n", filename.c_str(), (long)sizeof(data), current.size);
			    } 
			    else if  (current.count2!=current.count) {
			      fprintf(stderr,"ERROR> IPC FILE %s COUNT SLICING: expected %ld, got %ld\n", filename.c_str(), current.count, current.count2);
			    } else {
			      //if (0!=memcmp(&current,&last,sizeof(last))) // compare actual data
			      data=current.data;
	  				changed=true;
	  		  }
	  	  }
			  last=current;
			}
			fclose(f);
		}
		return changed;
	}
};



#endif


