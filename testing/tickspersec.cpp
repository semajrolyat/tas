#include <stdio.h>
#include <unistd.h>

int main( int argc, char* argv[] ) {

    int tickspersec = sysconf(_SC_CLK_TCK);

    printf( "Ticks per sec: %d\n", tickspersec );
}
