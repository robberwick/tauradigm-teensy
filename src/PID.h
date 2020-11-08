#ifndef _PID__H_
#define _PID__H_

class PID {
    public:
      float pGain;
      float dGain;
      float iGain;
      float update(float targetValue, float currentValue, float previousValue, float timeInterval);
    private:
      float cumulativeError;
};

#endif //_PID__H_