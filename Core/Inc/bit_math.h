#ifndef BIT_MATH_H
#define BIT_MATH_H

#define GET_BIT(REG, BitNo)     ((REG >> BitNo) & (1))
#define SET_BIT(REG, BitNo)     (REG |= (1 << BitNo))
#define CLR_BIT(REG, BitNo)     (REG &= ~(1 << BitNo))
#define TOG_BIT(REG, N)         (REG ^= (1 << BitNo))

#endif
