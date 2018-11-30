#ifndef Pipe_H
#define Pipe_H

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdlib>

#define FIFO_W "/tmp/fifo_c-p"
#define FIFO_R "/tmp/fifo_p-c"

#define MAX_BUF 1024


using namespace std;

struct pose{
	int id;
	double time;
	double x;
	double y;
	double z;
};


class Pipe{
public:
	Pipe();
	~Pipe();
	void init();
	void SendMessage(pose d);
	char* ReceiveMessage();

private:
	int Senderfd;
	int Readerfd;	

};




#endif /*Pipe_H*/