#include <iostream>
using namespace std;
#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16
char CHR[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

unsigned short base = OCT;

void WRITE(char c){
    cout<<c;
}

void getrem(int num, unsigned short base = DEC)
{
	if(num>0){
        getrem(num/base, base);
        int rem = num%base;
        WRITE(CHR[rem]);
    }
}

int main() {
    int num;
    cin>>num;
    if(num == 0) WRITE(CHR[0]);
	else if(num<0){
    	num *= -1;
    	WRITE('-');
    	getrem(num, base);
    }
    else if(num>0){
    	getrem(num, base);
    }     
	return 0;
}

