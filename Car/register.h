#ifndef REGISTER_H
#define REGISTER_H

/* Wrapper functions */
#define RESET(x)        _XRS(x)     // Reset the given PORT bit
#define SET(x)          _XS(x)      // Set the given PORT bit
#define SET_INPUT(x)    _XSI(x)     // Reset the given DDR bit
#define SET_OUTPUT(x)   _XS(x)      // Set the given DDR bit
#define IS_SET(x)          _XR(x)      // Get the value of the given PIN

/* Register Bit operations */
#define _XRS(x,y)   PORT(x) &= ~(1 << y)
#define _XS(x,y)    PORT(x) |=  (1 << y)
#define _XR(x,y)    (PIN(x) >> y) & 1
#define _XSI(x,y)   DDR(x) &= ~(1 << y)
#define _XSO(x,y)   DDR(x) |= (1 << y)

/* Register Names */
#define PORT(x) PORT ## x
#define DDR(x)  DDR ## x
#define PIN(x)  PIN ## x

/* Register Bits */
#define P(x,y)  P ## x ## y

#endif // REGISTER_H
