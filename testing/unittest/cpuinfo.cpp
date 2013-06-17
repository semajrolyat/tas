
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>


int main(int argc, char **argv) {
  FILE *cmdline = fopen("/proc/cpuinfo", "rb");
  char *arg = 0;
  size_t size = 0;
  while(getdelim(&arg, &size, 0, cmdline) != -1) {

    char* tofree;

    //if (arg != NULL) {

      tofree = arg;
      char *field = strsep(&arg, ":");
      char *value = strsep(&arg, ":");

      if( field != NULL ) {
        std::cout << field;
        if( value != NULL ) {
            std::cout << ":" << value; // << "\n";
        } else {
            std::cout << "\n";
        }
      }
      //free(tofree);
    //}

    //char str[1024];
    //strcpy( str, arg );

      /*
    char *delimiter = ":";
    char *field = strtok( arg, delimiter );
    char *value = strtok( NULL, delimiter );

    std::cout << field << ":" << value; // << "\n";
    */

    //puts(arg);
  }
  //free(arg);
  fclose(cmdline);
  return 0;

  /*
  FILE *cmdline = fopen("/proc/cpuinfo", "rb");
  char *arg = 0;
  size_t size = 0;

  char *field, *value;

  while( 1 ) {

    int n = getline( &arg, &size, cmdline );
    if( feof( cmdline ) ) break;
    //if( n == -1 ) {
    //    if( feof( cmdline ) ) break;
    //}

    char *delimiter = ":";
    char *field = strtok( arg, delimiter );
    char *value = strtok( NULL, delimiter );

    std::cout << field << ":" << value; // << "\n";
  }
  free(arg);
  fclose(cmdline);
  return 0;
  */
}
