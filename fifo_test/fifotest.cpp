#include <iostream>
#include "Pipe.h"

using namespace std;

int main()
{
	printf("hellowww \n");
	Data* d = new Data();
	d->id = 1;
	d->time =1.2;
	d->value =2.4;
	cout<<d->id<<endl;
	Pipe p;
	p.init();
	p.SendMessage(d);
	char * str = p.ReceiveMessage();
	cout<<str<<endl;

	return 0;
}