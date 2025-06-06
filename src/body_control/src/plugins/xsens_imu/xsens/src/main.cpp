#include <iostream>
#include <fstream>
#include <string>
#include <vector> 
#include <stdio.h>  
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <list>
#include <iomanip>

#include "../include/xsense/xsense.h"

#include "../include/broccoli/core/Time.hpp"

using namespace std;
using namespace broccoli::core;
bool dataLog(Eigen::VectorXd &v, std::ofstream &f);
int main(){
    Eigen::Matrix3d R_xense;
    Eigen::Vector3d euler_w, euler_w2, euler_b;

    Eigen::VectorXd imuData=Eigen::VectorXd::Zero(13);
	
    xsense_init(&imuData);
	xsense_start();

    Time start_time;
    Time period(0,2500000);
    Time sleep2Time;
    Time timer;
	Time time1;
    timespec sleep2Time_spec;

    int simCnt = 0;
    double timeSim = 0.0;
    double timeStep = 0.005;

    std::ofstream foutData;
    foutData.open("datacollection.txt",std::ios::out);
    Eigen::VectorXd dataL = Eigen::VectorXd::Zero(30);
    while (1)
    {   
		start_time = timer.currentTime();

        if(simCnt%40==0){
            cout << "rpy" << imuData.head(3).transpose()/M_PI*180.0 << endl;
            cout << "gyro" << imuData.segment(3,3).transpose() << endl;
		    cout << "acc" << imuData.segment(6,3).transpose() << endl;
        }
		
		
        time1 = timer.currentTime()-start_time;
        simCnt += 1;
        timeSim =  simCnt*timeStep;
        //
        dataL[0] = timeSim;
        dataL[1] = start_time.m_nanoSeconds;
        dataL[2] = time1.m_nanoSeconds;
        dataL.segment(3,9) = imuData.head(9);
	
        sleep2Time = start_time + period;
        sleep2Time_spec = sleep2Time.toTimeSpec();
        clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&(sleep2Time_spec),NULL);
    }
    
    foutData.close();
    return 0;
}
//logData
bool dataLog(Eigen::VectorXd &v, std::ofstream &f){
    for(int i=0;i<v.size();i++){
          f<<v[i]<<" ";
        }
        f<<std::endl;
    return true;
}