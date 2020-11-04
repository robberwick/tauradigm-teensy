
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
        {420, 0},  //was 550, 0  1200
        {950, -100}, //was 50
        {1070, -300},
        {975, -400},
        {625, -425}, 
        {480, -590},
        {250, -565},
        {150, -625},
        {100, -700},
        {150, -850},
        {300, -900},
        {950, -1000}
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