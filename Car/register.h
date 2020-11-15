#ifndef REGISTER_H
#define REGISTER_H

#define true    1
#define false   0

/* Wrapper functions */
#define RESET(x)        _XRS(x)     // Reset the given PORT bit
#define SET(x)          _XS(x)      // Set the given PORT bit
#define SET_INPUT(x)    _XSI(x)     // Reset the given DDR bit
#define SET_OUTPUT(x)   _XSO(x)      // Set the given DDR bit
#define IS_SET(x)       _XR(x)      // Is the input PIN set?

#define PORT(x)     _port(x)
#define DDR(x)      _ddr(x)
#define PIN(x)      _pin(x)

/* Register Bit operations */
#define _XRS(x,y)   PORT(x) &= ~(1 << y)
#define _XS(x,y)    PORT(x) |=  (1 << y)
#define _XR(x,y)    (PIN(x) & (1 << y)) != 0
#define _XSI(x,y)   DDR(x) &= ~(1 << y)
#define _XSO(x,y)   DDR(x) |= (1 << y)

/* Register Names */
#define _port(x)    PORT ## x
#define _ddr(x)     DDR ## x
#define _pin(x)    PIN ## x

#endif // REGISTER_H
