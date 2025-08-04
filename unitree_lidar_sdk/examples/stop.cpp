/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_lidar_sdk.h"

using namespace unitree_lidar_sdk;

int main(){

  // Initialize Lidar Object
  UnitreeLidarReader* lreader = createUnitreeLidarReader();
  int cloud_scan_num = 18;
  std::string port_name = "/dev/ttyUSB0";

  if ( lreader->initialize(cloud_scan_num, port_name) ){
    printf("Unilidar initialization failed! Exit here!\n");
    exit(-1);
  }else{
    printf("Unilidar initialization succeed!\n");
  }

  // Set Lidar Working Mode
  printf("Set Lidar working mode to: STANDBY ... \n");
  lreader->setLidarWorkingMode(STANDBY);
  sleep(5);
 
  return 0;
}