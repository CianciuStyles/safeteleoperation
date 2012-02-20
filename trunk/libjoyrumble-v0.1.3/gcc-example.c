/*
* libjoyrumble example
*
* by Cleber de Mattos Casali <clebercasali@yahoo.com.br>
*
*
* Compile with:
*
* gcc gcc-example.c -o gcc-example -ldl
*
* If you're running on x86_64 (64 bits), and want to compile the library for x86 (32 bits), add the "-m32" flag to gcc.
*
*/

/*  Includes */
#include <stdio.h>
#include <stdlib.h>
#include<unistd.h>
#include <dlfcn.h>

/* Define the function arguments */
typedef int (*joyrumblefntype) (int joynumber, int strong, int weak, int duration);


int main(int argc, char *argv[]){

   void *lib_handle;
   joyrumblefntype joyrumblefn;
   char *error;

   /* Load the library */
   lib_handle = dlopen("./libjoyrumble.so", RTLD_LAZY);
   if (!lib_handle) 
   {
      fprintf(stderr, "%s\n", dlerror());
      exit(1);
   }

   
   /* Get the function pointer */
   joyrumblefn = dlsym(lib_handle, "joyrumble");
   if ((error = dlerror()) != NULL)  
   {
      fprintf(stderr, "%s\n", error);
      exit(1);
   }

   /* Define the joyrumble function to simplify */
   #define joyrumble (*joyrumblefn)

   printf("Will rumble now...\n");
   
   /* Make it rumble */
   joyrumble(2,100,100,1000);

   sleep(2);

   printf("Ok!\n");

   return 0;
}
