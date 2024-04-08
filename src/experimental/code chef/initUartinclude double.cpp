#include <iostream>
using namespace std;

#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16

char CHR[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
bool flag = true;  //val has not come yet
int res = 4;

void UARTgetrem(long num, unsigned short base = DEC, bool ch = false)
{
    if(ch){
        if(num>0 || res>1){
            res--;
            UARTgetrem(num/base, base, true);
            int rem = num%base;
            cout<<CHR[rem];
        }
    }
    else {
    	if(num>0){
            UARTgetrem(num/base, base);
            int rem = num%base;
            cout<<CHR[rem];
        }
    }
}

void UARTgetremf(long num, unsigned short base = DEC)
{
    int rem = num%base;
    while(flag){
        if(rem == 0){
            num /= base;
            rem = num%base;
            res--;
        }
        else{
            flag = false;
        }
    }
	if(num>0){
        UARTgetrem(num/base, base, true);
        rem = num%base;
        cout<<CHR[rem];
    }
}

int main() {
    double val = 1290.0100;
    int base = DEC;
    long num = val;
    
    if(num == 0) cout<<CHR[0];
	else if(num<0){
    	num *= -1;
    	cout<<'-';
    	UARTgetrem(num, base);
    }
    else if(num>0){
    	UARTgetrem(num, base);
    }     
    
    val*=10000;
    num = val - 10000*num;
    if(num>0)cout<<'.';
    if(num>0){
    	UARTgetremf(num);
    }     
    
}
