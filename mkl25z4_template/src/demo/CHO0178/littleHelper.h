#ifndef LITTLE_HELPER_H
#define LITTLE_HELPER_H



#define setBitInRegister(REG, pos)  						REG |= 1 << pos
#define clearBitInRegister(REG, pos)  						REG &= ~(1 << pos)
#define toogleBitInRegister(REG, pos)  						REG ^= 1 << pos
#define changeMultipleBitInRegister(REG, DATA, MASK, pos)   REG = (DATA << pos) | (REG & (~(MASK & REG)))









#endif
