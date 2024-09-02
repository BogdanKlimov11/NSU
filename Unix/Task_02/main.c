#include <sys/types.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

int main()
{
    putenv("TZ=America/Los_Angeles");
    time_t now;
    struct tm *sp;

    (void)time(&now);

    printf("%s", ctime(&now));

    sp=localtime(&now);
    printf("%d/%d/%02d %d:%02d %s\n", sp->tm_mon + 1, sp->tm_mday,
           sp->tm_year, sp->tm_hour, sp->tm_min, tzname[sp->tm_isdst]);
    
    exit(0);
}
