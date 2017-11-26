//
//  helpers.cpp
//  path_planning
//
//  Created by Emil Balcu on 26.11.17.
//



#include <math.h>


float logistic(float x){
    /*
    A function that returns a value between 0 and 1 for x in the
        range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
     Useful for cost functions.
     */
    
    return 2.0 / (1 + exp(-x)) - 1.0;
    
}

