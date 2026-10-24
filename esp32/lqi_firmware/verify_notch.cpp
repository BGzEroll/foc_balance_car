#include "common_notch.h"
#include <iostream>
#include <iomanip>
int main(){common_notch::state s;float input;std::cout<<std::setprecision(9);while(std::cin>>input)std::cout<<s.update(input)<<'\n';}
