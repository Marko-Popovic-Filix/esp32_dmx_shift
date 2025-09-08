#include "utils.h"
#include <string.h>
static int hexval(char c){ if(c>='0'&&c<='9')return c-'0'; if(c>='a'&&c<='f')return c-'a'+10; if(c>='A'&&c<='F')return c-'A'+10; return -1; }
int util_url_decode(char *s){
    size_t len=strlen(s), w=0;
    for(size_t r=0;r<len;++r){
        if(s[r]=='+'){ s[w++]=' '; }
        else if(s[r]=='%' && r+2<len){
            int h=hexval(s[r+1]), l=hexval(s[r+2]); if(h<0||l<0) return -1;
            s[w++]=(char)((h<<4)|l); r+=2;
        } else { s[w++]=s[r]; }
    } s[w]=0; return (int)w;
}
