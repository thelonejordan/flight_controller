#include <iostream>
using namespace std;

void WRITE(char c[]){
    int i = 0;
	while(c[i] != NULL){
	    cout<<c[i];
	    i++;
	}
}

int main() {
	// your code goes here
	
	WRITE("Jyotirmaya");
	return 0;
}
