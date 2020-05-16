#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <termio.h>
#include <termios.h>


#include <signal.h>
#include <pwd.h>

#define ESC "\033"


#define TIMEOUT         10


char *myname;

char getsize[] = ESC "7"  ESC "[r" ESC "[999;999H" ESC "[6n";
char restore[] = ESC "8";

struct termios tioorig;

char size[] = ESC "[%d;%dR";

int tty;
FILE *ttyfp;

static void onintr (int sig);
static void resize_timeout (int sig);
static void Usage (void);
static void readstring (FILE *fp, char *buf, char *str);

char *
x_basename(char *name)
{
   char *cp;

   cp = strrchr(name, '/');
   return (cp ? cp + 1 : name);
}


/*
  tells tty driver to reflect current screen size
 */

int
main (int argc, char **argv)
{

       int rows, cols;
       struct termios tio;
       char buf[BUFSIZ];
       struct winsize ws;
       char *name_of_tty;

       myname = x_basename(argv[0]);

       if (argc > 1) Usage();

       name_of_tty = "/dev/tty";

       if ((ttyfp = fopen (name_of_tty, "r+")) == NULL) {
           fprintf (stderr, "%s:  can't open terminal %s\n",
                    myname, name_of_tty);
           exit (1);
       }
       tty = fileno(ttyfp);

       tcgetattr(tty, &tioorig);
       tio = tioorig;
       tio.c_iflag &= ~ICRNL;
       tio.c_lflag &= ~(ICANON | ECHO);
       tio.c_cflag |= CS8;
       tio.c_cc[VMIN] = 6;
       tio.c_cc[VTIME] = 1;
       signal(SIGINT, onintr);
       signal(SIGQUIT, onintr);
       signal(SIGTERM, onintr);
       tcsetattr(tty, TCSADRAIN, &tio);

       write(tty, getsize, strlen(getsize));
       readstring(ttyfp, buf, size);
       if(sscanf (buf, size, &rows, &cols) != 2) {
               fprintf(stderr, "%s: Can't get rows and columns\r\n", myname);
               onintr(0);
       }
       write(tty, restore, strlen(restore));

       if (ioctl (tty, TIOCGWINSZ, &ws) != -1) {
           /* we don't have any way of directly finding out
              the current height & width of the window in pixels.  We try
              our best by computing the font height and width from the "old"
              struct winsize values, and multiplying by these ratios...*/
           if (ws.ws_col != 0)
               ws.ws_xpixel = cols * (ws.ws_xpixel / ws.ws_col);
           if (ws.ws_row != 0)
               ws.ws_ypixel = rows * (ws.ws_ypixel / ws.ws_row);
           ws.ws_row = rows;
           ws.ws_col = cols;
           ioctl (tty, TIOCSWINSZ, &ws);
       }

       tcsetattr(tty, TCSADRAIN, &tioorig);
       signal(SIGINT, SIG_DFL);
       signal(SIGQUIT, SIG_DFL);
       signal(SIGTERM, SIG_DFL);

       exit(0);
}


static void
readstring(register FILE *fp, register char *buf, char *str)
{
       register int last, c;

       signal(SIGALRM, resize_timeout);
       alarm (TIMEOUT);
       if ((c = getc(fp)) == 0233) {   /* meta-escape, CSI */
               *buf++ = c = ESC[0];
               *buf++ = '[';
       } else {
               *buf++ = c;
       }
       if(c != *str) {
               fprintf(stderr, "%s: unknown character, exiting.\r\n", myname);
               onintr(0);
       }
       last = str[strlen(str) - 1];
       while((*buf++ = getc(fp)) != last)
           ;
       alarm (0);
       *buf = 0;
}

static void
Usage(void)
{
       fprintf(stderr,
        "Usage: %s\n"
        "   sets size via ioctl\n", myname);
       exit(1);
}

static void
resize_timeout(int sig)
{
       fprintf(stderr, "\n%s: timeout occurred\r\n", myname);
       onintr(sig);
}

/* ARGSUSED */
static void
onintr(int sig)
{
       tcsetattr (tty, TCSADRAIN, &tioorig);
       exit(1);
}
