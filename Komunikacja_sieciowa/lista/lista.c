#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <string.h>

int main(void)
{
  DIR *dp;
  struct dirent *ep;
  int len;

  dp = opendir ("/Users/aleksandersteplewski/Pictures");
  if (dp != NULL)
    {
      while (ep = readdir (dp))
	    {
	      len=strlen(ep->d_name);
     	  char *last_four=&ep->d_name[len-4];
	      char *last_three=&ep->d_name[len-3];
	      char *last_two=&ep->d_name[len-2];
	      char *last_one=&ep->d_name[len-1];
	      if((*last_four=='.')&&(*last_three=='j')&&(*last_two=='p')&&(*last_one=='g'))
	      {
        	puts (ep->d_name);
	      }
	    }
      (void) closedir (dp);
    }
  else {
    perror ("Couldn't open the directory");
  }
  return 0;
}