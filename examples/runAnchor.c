#include "anchor.h"

int main()
 {
    setupAnchor();
    pairWithTag(1.23,3.46);
    send_int_VLC(23621, 2); //Send Interger every 2 seconds
    runAnchor();
    return 0;
 }
