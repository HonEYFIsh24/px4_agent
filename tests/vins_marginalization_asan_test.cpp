#include "marginalization_factor.h"
#include "imu_bias_prior.h"
#include <iostream>
int main(){for(int i=0;i<100;++i){double state[9]={};MarginalizationInfo info;
info.addResidualBlockInfo(new ResidualBlockInfo(new ImuBiasPrior(.196,.0087),nullptr,{state},{0}));
info.preMarginalize();}std::cout<<"PASS: native new[] preMarginalize allocations/destructor (100 iterations)\n";}
