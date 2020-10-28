
struct Pose {
    float x = 0;
    float y = 0;
    float heading = 0;
};


//#define RH_SQUARE // right hand square
#define LH_SQUARE // left hand square

// right hand square
#ifdef RH_SQUARE
    Pose route[] = { 
        {600, 0},
        {600, 600},
        {0, 600},
        {0, 0}
    };
#endif

// left hand square
#ifdef LH_SQUARE
    Pose route[] = { 
        {600, 0},
        {600, -600},
        {0, -600},
        {0, 0}
    };
#endif
