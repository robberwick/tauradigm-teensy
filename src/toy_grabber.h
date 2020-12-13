#include "jaws.h"
#include "lifter.h"

class ToyGrabber {
   public:
    ToyGrabber(uint8_t jawsPin, uint8_t lifterPin);
    ~ToyGrabber();

    enum class State {
        WAIT = 0,
        OPENING = 1,
        GRABBING = 2,
        LOWERING = 3,
        LIFTING = 4,
    };

    enum class Command {
        STOP,
        RUN
    };

    State getState();
    void run();
    void stop();

    void update();

   private:
    Jaws _jaws;
    Lifter _lifter;

    State _currentState;
    Command _requestedCommand;
};
