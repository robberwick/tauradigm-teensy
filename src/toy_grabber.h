#include "jaws.h"
#include "lifter.h"

class ToyGrabber {
   public:
    ToyGrabber(uint8_t jawsPin, uint8_t lifterPin);
    ~ToyGrabber();

    enum class State {
        WAIT,
        OPENING,
        GRABBING,
        LOWERING,
        LIFTING,
        DEPOSITING,
    };

    enum class Command {
        STOP,
        PICKUP,
        DEPOSIT
    };

    State getState();
    void begin();
    void pickup();
    void deposit();

    void update();

   private:
    Jaws _jaws;
    Lifter _lifter;

    State _currentState;
    Command _requestedCommand;
};
