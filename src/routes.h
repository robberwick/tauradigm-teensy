
struct Pose {
    float x = 0;
    float y = 0;
    float heading = 0;
};


//#define RH_SQUARE // right hand square
//#define LH_SQUARE // left hand square
#define GARDEN_PATH

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


// right hand square
#ifdef GARDEN_PATH
    Pose route[] = { 
        {1123, -141},
        {1197, -428},
        {975, -589},
        {779, -532}, 
        {485, -735},
        {201, -727},
        {70, -1008},
        {325, -1171},
        {1354, -1354}
    };
#endif