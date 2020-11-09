
#include "types.h"

//#define RH_SQUARE // right hand square
#define LH_SQUARE // left hand square
//#define GARDEN_PATH // Up The Garden Path challenge route
//#define TIDY_TOYS // Tidy Up The Toys challenge route

// right hand square
#ifdef RH_SQUARE
    Pose route[] = { 
        {0, 600, 0},
        {0, 600, 600},
        {0, 0, 600},
        {0, 0, 0}
    };
#endif

// left hand square
#ifdef LH_SQUARE
    Pose route[] = { 
        {0, 600, 0},
        {0, 600, -600},
        {0, 0, -600},
        {0, 0, 0}
    };
#endif


// Up the Garden Path challenge route
#ifdef GARDEN_PATH
    Pose route[] = {
        {0, 420, 0},  //was 550, 0  1200
        {0, 950, -100}, //was 50
        {0, 1070, -300},
        {0, 975, -400},
        {0, 625, -425}, 
        {0, 480, -590},
        {0, 250, -565},
        {0, 150, -625},
        {0, 100, -700},
        {0, 150, -850},
        {0, 300, -900},
        {0, 950, -1000}
    };
#endif


// Tidy Up The Toys challenge route
#ifdef TIDY_TOYS
    Pose route[] = { 
        {0, 800, 60},
        {0, 820, -400},
        {0, 750, 460},
        {0, 450, -520}
    };
#endif
