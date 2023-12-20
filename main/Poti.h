#ifndef POTI_H
#define POTI_H

#if !defined(NOSENSORS)
#include "I2Cbus.hpp"
#endif

// Template for digital Poti Interface

class Poti
{
public:
  virtual ~Poti() {};
  virtual bool begin() = 0;
#if !defined(NOSENSORS)
  virtual void setBus( I2C_t *theBus ) = 0;
#endif
  virtual bool readVolume( float& val ) = 0;
  virtual bool writeVolume( float val ) = 0;
  virtual bool haveDevice() = 0;
private:
  virtual int  getRange() = 0;
  virtual float getInvRange() = 0;
  virtual bool readWiper( int& val ) = 0;
  virtual bool writeWiper( int val ) = 0;
  virtual int  getStep() = 0;
};

#endif
