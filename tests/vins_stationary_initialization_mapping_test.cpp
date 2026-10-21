// Tests the production startup validation helpers, not a second implementation.
#include "stationary_initialization.h"
#include <iostream>
#include <limits>
#include <stdexcept>
static void expect(bool okay,const char *why) { if(!okay) throw std::runtime_error(why); }
int main() {
 try {
  std::vector<double> headers,spans;
  for(int i=0;i<11;++i) { headers.push_back(20.+.1*i);spans.push_back(i?.1:0); }
  std::string reason;
  expect(validateInitialHeaderEdges(headers,headers,spans,spans,reason),"valid original adjacent edges rejected");
  auto keys=headers;keys.erase(keys.begin());
  expect(!validateInitialHeaderEdges(headers,keys,spans,spans,reason),"missing map header accepted");
  keys=headers;keys.insert(keys.begin(),19.9);
  expect(!validateInitialHeaderEdges(headers,keys,spans,spans,reason),"extra OLD map header accepted");
  keys=headers;keys[4]+=.001;
  expect(!validateInitialHeaderEdges(headers,keys,spans,spans,reason),"unmapped source header accepted");
  auto broken=headers;broken[5]=broken[4];
  expect(!validateInitialHeaderEdges(broken,broken,spans,spans,reason),"copied prediction slot mistaken for captured source");
  broken=headers;broken[5]=std::numeric_limits<double>::quiet_NaN();
  expect(!validateInitialHeaderEdges(broken,broken,spans,spans,reason),"nonfinite source header accepted");
  auto edge=spans;edge[6]=.2;
  expect(!validateInitialHeaderEdges(headers,headers,edge,spans,reason),"merged edge inconsistent with headers accepted");
  edge=spans;edge[6]=-1;
  expect(!validateInitialHeaderEdges(headers,headers,spans,edge,reason),"missing ImageFrame edge accepted");
  edge=spans;edge[6]=std::numeric_limits<double>::infinity();
  expect(!validateInitialHeaderEdges(headers,headers,edge,spans,reason),"nonfinite raw IMU span accepted");
  bool frames[11],intervals[11];for(int i=0;i<11;++i) frames[i]=intervals[i]=true;
  intervals[0]=false; // no preceding interval to frame zero
  expect(confirmedStationaryInitialWindow(frames,intervals,10,true,true,reason),"full continuous confirmation rejected");
  frames[0]=false;
  expect(!confirmedStationaryInitialWindow(frames,intervals,10,true,true,reason),"unconfirmed oldest frame accepted");
  frames[0]=true;intervals[5]=false;
  expect(!confirmedStationaryInitialWindow(frames,intervals,10,true,true,reason),"motion history between static endpoints accepted");
  intervals[5]=true;
  expect(!confirmedStationaryInitialWindow(frames,intervals,10,false,true,reason),"lost current source gate accepted");
  expect(!confirmedStationaryInitialWindow(frames,intervals,10,true,false,reason),"unconfirmed current source accepted");
  std::cout<<"PASS: production source mapping rejects missing/extra/duplicate headers, wrong raw edges and broken stationary continuity\n";
  return 0;
 } catch(const std::exception &e) { std::cerr<<"FAIL: "<<e.what()<<'\n';return 1; }
}
