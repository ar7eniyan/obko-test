#ifndef INCLUDE_TOOLS_H
#define INCLUDE_TOOLS_H

#define SET_RCC_xxxxEN(rccreg, mask) \
do { \
    __IO uint32_t tmpreg; \
    (rccreg) |= (mask); \
    tmpreg = (rccreg); \
    (void)tmpreg; \
} while (0)

#endif  // #ifdef INCLUDE_TOOLS_H

