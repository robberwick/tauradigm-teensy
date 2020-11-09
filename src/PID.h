#ifndef _PID__H_
#define _PID__H_

#include "utils.h"

class PID {
    public:
      float pGain;
      float dGain;
      float iGain;
      bool piWrapping = false;
      float update(float targetValue, float currentValue, float previousValue, float timeInterval);
    private:
      float cumulativeError;
};

#endif //_PID__H_