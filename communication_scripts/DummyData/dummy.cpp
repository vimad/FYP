#include "Pipe.h"
#include <iostream>
#include <sys/time.h>

using namespace std;

double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

double wait(double seconds, double last_t)
{
	double new_t = tic();
	while((new_t-last_t)<seconds)
	{
		new_t = tic();
	}

	return new_t;
}

int main(){

	cout<<"program start"<<endl;

	Pipe pipe;
	pipe.init();

	double x[] = {3.00,2.50,2.00,1.50,1.10,0.70,0.40,0.20,0.15,0.10,0.20,0.10,0.10,0.40,0.20,0.10,0.10,0.05,0.05};
	double y[] = {2.00,1.60,1.20,0.80,0.60,0.40,0.30,0.20,0.15,0.05,0.10,0.05,0.10,0.20,0.10,0.05,0.05,0.10,0.10};
	double z[] = {8.00,8.00,8.00,8.00,8.00,8.00,8.00,8.00,8.00,8.00,6.00,6.00,4.00,2.50,2.50,2.50,1.00,0.50,0.20};
	int len = sizeof(x)/sizeof(*x);
	int i = 0;
	double last_t = tic();

	while(i < len){

		cout<<i<<endl;

		pose p;
		p.id = 1;
		p.time = tic();
		p.x = x[i];
		p.y = y[i];
		p.z = z[i];

		pipe.SendMessage(p);
		
		last_t = wait(0.5,last_t);
		i++;
	}

	return 0;
}
