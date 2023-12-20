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

  bool readVolume( float& val );
  bool writeVolume( float val );
  bool haveDevice();

private:
  bool readWiper( int& val );
  bool writeWiper( int val );
#define SOFTPOTIRANGE 127
  inline int  getRange() { return SOFTPOTIRANGE; };
  inline float getInvRange() { return (1.0/SOFTPOTIRANGE); };
  inline int  getStep() { return 32; };

  int wiper;  // only bit 0..7 supported
};

#endif

#endif
