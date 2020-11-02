
struct Pose {
    float x = 0;
    float y = 0;
    float heading = 0;
};


//#define RH_SQUARE // right hand square
//#define LH_SQUARE // left hand square
#define GARDEN_PATH // Up The Garden Path challenge route
//#define TIDY_TOYS // Tidy Up The Toys challenge route

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


// Up the Garden Path challenge route
#ifdef GARDEN_PATH
    Pose route[] = { 
        {1023, -141},
        {1050, -350},
        {875, -500},
        {679, -532}, 
        {385, -735},
        {201, -727},
        {70, -808},
        {325, -971},
        {1154, -1154}
    };
#endif


// Tidy Up The Toys challenge route
#ifdef TIDY_TOYS
    Pose route[] = { 
        {800, 60},
        {820, -400},
        {750, 460},
        {450, -520}
    };
#endif