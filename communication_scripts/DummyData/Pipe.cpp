#include "Pipe.h"

Pipe::Pipe()
{

}

Pipe::~Pipe(){
	close(Senderfd);
	close(Readerfd);
}

void Pipe::init()
{
	//printf("init fifo\n");
	mkfifo(FIFO_W, 0666);
	mkfifo(FIFO_R, 0666);
}


void Pipe::SendMessage(pose d)
{
	//printf("sending message... \n");
	Senderfd = open(FIFO_W,O_WRONLY);
	ssize_t bufsz = snprintf(NULL,0,"%d,%.02f,%.02f,%.02f,%.02f",d.id, d.time, d.x, d.y, d.z);
	char* s = (char *)malloc(bufsz + 1);
	snprintf(s,bufsz,"%d,%.02f,%.02f,%.02f,%.02f",d.id, d.time, d.x, d.y, d.z);
	write(Senderfd,s,bufsz);
	free(s);
	close(Senderfd);
}

char* Pipe::ReceiveMessage()
{
	Readerfd = open(FIFO_R, O_RDONLY);
	char* buf= (char *)malloc(MAX_BUF);
    if(read(Readerfd, buf, MAX_BUF) < 0)
    {
    	printf("Error Occured.. not received \n");
    	exit(-1);
    }
    close(Readerfd);
    return buf;
}
