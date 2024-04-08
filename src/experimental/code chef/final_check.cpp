#include <iostream>
using namespace std;
#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16

char CHR[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

void WRITE(char c){
    cout<<c;
}

void WRITE(char c[]){
    int i = 0;
	while(c[i] != NULL){
	    cout<<c[i];
	    i++;
	}
}


void getrem(int num, unsigned short base = DEC)
{
	if(num>0){
        getrem(num/base, base);
        int rem = num%base;
        WRITE(CHR[rem]);
    }
}

void print(int num, unsigned short base = DEC){
	if(num == 0) WRITE(CHR[0]);
	else if(num<0){
    	num *= -1;
    	WRITE('-');
    	getrem(num, base);
    }
    else if(num>0){
    	getrem(num, base);
    }     
}

void print(char str){
    WRITE(str);
		
}

void print(char str[]){
	unsigned int i = 0;
	while(str[i] != NULL){
		WRITE(str[i]);
		i++;
	}
}

void println(int num, unsigned short base = DEC){
    print(num, base);
    WRITE('\n');
}

void println(char str){
	print(str);
	WRITE('\n');
}

void println(char str[]){
	print(str);
	WRITE('\n');
}

int main() {
	// your code goes here
	println('a');
	print("Arduino");
	return 0;
}
