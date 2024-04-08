#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16

void WRITE();  //WRITE takes only one byte char and prints it on the screen

char CHR[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

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

void getrem(int num, unsigned short base = DEC)
{
	if(num>0){
        getrem(num/base, base);
        int rem = num%base;
        WRITE(CHR[rem]);
    }
}