#ifndef _pcUART_h
#define _pcUART_h

#define BIN 2
#define OCT 8
#define DEC 10
#define HEX 16

class Serial
{
	private :
	    char CHR[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
	    void WRITE(char c);
	    void getrem(long num, unsigned long base = DEC);

	public :
	    void begin(unsigned int baud_rate);
        void print(long num, unsigned short base = DEC);
        void print(char str);
        void print(char str[]);
        void println(long num, unsigned short base = DEC);
        void println(char str);
        void println(char str[]);
};
#endif