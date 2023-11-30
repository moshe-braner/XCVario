#ifndef SOFTPOTI_H
#define SOFTPOTI_H

#if defined(NOSENSORS)

#include "Poti.h"

// include these just so we can un-virtualize the setBus() function
//#include "I2C.h"
//#include "I2Cbus.hpp"

// coarse volume adjustment via the DAC scale factor

class SoftPoti: public Poti
{
public:
  /*  Creates instance  */
  SoftPoti();

  bool begin();

//  void setBus( I2C_t *theBus ) { };

  /*  Destroys instance  */
  ~SoftPoti();

  bool readWiper( uint16_t& val );
  bool writeWiper( uint16_t val );
  bool incWiper();
  bool decWiper();
  bool haveDevice();
  inline int  getRange() { return 127; };  // 7 bit 0..127
  inline int  getStep() { return 32; };

private:
  uint16_t wiper;  // only bit 0..7 supported
};

#endif

#endif
