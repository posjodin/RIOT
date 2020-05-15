TAGDIRS="sys core drivers/periph_common/ drivers/at"
TAGFILES="drivers/include/periph/uart.h drivers/include/at.h"
#find  ${TAGDIRS} -name '*.[ch]'
find  ${TAGDIRS} -name '*.[ch]' | etags -
etags -a ${TAGFILES}

