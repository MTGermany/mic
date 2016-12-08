//(arne may 05)
//some remarks about the headers:
//c headers do have ending *.h or 
//a "c" instead at the beginning: 
// <stdio.h> --> <cstdio>
//they differ in some namespace 
//but they are identified if one just write 
//"using namespace std;" after the includes!!!

// c
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// c++ 
#include <iostream>
#include <fstream>
using namespace std;


#include "ClassTemplate.h"

//standard constructor
ClassName::ClassName(){;} 

//standard deconstructor
ClassName::~ClassName(){;}

void ClassName::aPublicMethod(){
}
