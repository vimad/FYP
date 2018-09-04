
#include <fcntl.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>
#include <string.h>
using namespace std;
int main()
{
    int fd;
    char * myfifo = "/tmp/fifopipe";

    /* create the FIFO (named pipe) */
    mkfifo(myfifo, 0666);

    /* write message to the FIFO */
    fd = open(myfifo, O_WRONLY);
    char *msg;
    msg="This is the string to be reversed";
    write(fd, msg, strlen(msg)+1);
    close(fd);

    /* remove the FIFO */
    unlink(myfifo);

    return 0;
}