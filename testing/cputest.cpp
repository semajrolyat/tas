#include <cpu.h>

int main( void ) {

    cpuinfo_c cpuinfo;

    cpuinfo_err_e result = cpuinfo.load( );
    if( result != CPUINFO_ERROR_NONE ) {
	printf( "Failed to load cpuinfo: %d\n", result );
	return 1;
    }

    printf( "cpuinfo loaded\n" );

    for( std::vector<cpu_t>::iterator it = cpuinfo.cpus.begin(); it != cpuinfo.cpus.end(); it++ ) {
	printf( "cpu found: %d @ %f MHz\n", it->processor, it->mhz );
    }

    return 0;
}
