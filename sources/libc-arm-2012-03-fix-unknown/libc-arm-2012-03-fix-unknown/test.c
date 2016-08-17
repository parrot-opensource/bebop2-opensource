
#include <stdio.h>
#include <string.h>

int main(void) {
	char chld[] = "\r\n+CHLD: (0,1,1x,2,2x,3)\r\n\r\nOK\r\n\r\n+CIEV: 1,1\r\n\r\n+CIEV: 3,0\r\n";
	const char cCiev[]	= "\r\n+CIEV:";
	const char cOK[]	= "OK";
	char *res;
	const char *cres;
	chld[26] = 0;
	chld[27] = 0;

	res = strstr(chld, cCiev);
	fprintf(stderr, "test 1: %s\n", res == NULL ? "OK": "KO");

	res = strstr(chld, cOK);
	fprintf(stderr, "test 2: %s\n", res == NULL ? "OK": "KO");

	res = strstr(&chld[28], cOK);
	fprintf(stderr, "test 3: %s\n", res == &chld[28] ? "OK": "KO");

	cres = strstr("AAAAA\0 BBBB CCCC", "A B");
	fprintf(stderr, "test 4: %s\n", cres == NULL ? "OK": "KO");
	return 0;
}

