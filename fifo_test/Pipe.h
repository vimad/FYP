#ifndef Pipe_H
#define Pipe_H

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define FIFO_W "/tmp/fifo_c-p"
#define FIFO_R "/tmp/fifo_p-c"

#define MAX_BUF 1024


struct Data{
	int id;
	double time;
	double value;
};


class Pipe{
public:
	Pipe();
	~Pipe();
	void init();
	void SendMessage(Data* d);
	char* ReceiveMessage();

private:
	int Senderfd;
	int Readerfd;	

};

Pipe::Pipe()
{

}

Pipe::~Pipe(){
	close(Senderfd);
	close(Readerfd);
}

void Pipe::init()
{
	printf("init \n");
	mkfifo(FIFO_W, 0666);
	//mkfifo(FIFO_R, 0666);

	//Readerfd = open(FIFO_R, O_RDONLY);
	//Senderfd = open(FIFO_W,O_WRONLY);
}


void Pipe::SendMessage(Data* d)
{
	printf("sending message... \n");
	Senderfd = open(FIFO_W,O_WRONLY);
	ssize_t bufsz = snprintf(NULL,0,"%d,%.02f,%.02f",d->id,d->time,d->value);
	char* s = (char *)malloc(bufsz + 1);
	snprintf(s,bufsz,"%d,%.02f,%.02f",d->id,d->time,d->value);
	write(Senderfd,s,bufsz);
	free(s);
	close(Senderfd);
}

char* Pipe::ReceiveMessage()
{
	Readerfd = open(FIFO_R, O_RDONLY);
	char *buf = (char *)malloc(MAX_BUF);
    read(Readerfd, buf, MAX_BUF);
    return buf;
    close(Readerfd);
}

#endif /*Pipe_H*/