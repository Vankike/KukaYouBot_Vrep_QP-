
#include <vrepInit.h>
#include <eigen3/Eigen/Dense>
#include <iostream>

extern "C" {
#include "extApi.h"
    /*#include "extApiCustom.h" if you wanna use custom remote API functions! */
}
using namespace Eigen;

namespace VREP{

Simulador::Simulador(VectorXd Q,VectorXd Q2, bool wheelsON){

    Kwh = 9;
    Krot= 4.8;
    portNb = 19997;
    clientID = simxStart("127.0.0.1", portNb, true, true, 5000, 5);
    /// Configure the connection with Vrep

    //! Star the simulation in VREP.
    simxStartSimulation(clientID, simx_opmode_oneshot);
    //If the connection is ok then start the simulation.
    if (clientID == -1)
    {
        std::cout << "ERROR: Unable to connect with V-rep \n";
        std::cout << "Possible reason: First click on play in vrep \n";
        simxFinish(clientID);
        exit(1);
    }


    int valido1 = simxGetObjectHandle(clientID, "Joint1", &joint1, simx_opmode_blocking);
    int valido2 = simxGetObjectHandle(clientID, "Joint2", &joint2, simx_opmode_blocking);
    int valido3 = simxGetObjectHandle(clientID, "Joint3", &joint3, simx_opmode_blocking);
    int valido4 = simxGetObjectHandle(clientID, "Joint4", &joint4, simx_opmode_blocking);
    int valido5 = simxGetObjectHandle(clientID, "Joint5", &joint5, simx_opmode_blocking);

    int valido9 = simxGetObjectHandle(clientID, "Joint1#1", &joint1_2, simx_opmode_blocking);
    int valido10 = simxGetObjectHandle(clientID, "Joint2#1", &joint2_2, simx_opmode_blocking);
    int valido11 = simxGetObjectHandle(clientID, "Joint3#1", &joint3_2, simx_opmode_blocking);
    int valido12 = simxGetObjectHandle(clientID, "Joint4#1", &joint4_2, simx_opmode_blocking);
    int valido13 = simxGetObjectHandle(clientID, "Joint5#1", &joint5_2, simx_opmode_blocking);

    int valido17 = simxGetObjectHandle(clientID, "youBot", &baseX, simx_opmode_blocking);    
    int valido19 = simxGetObjectHandle(clientID, "youBot#1", &baseX2, simx_opmode_blocking);
    int valido21 = simxGetObjectHandle(clientID, "youBot_ref", &Ox, simx_opmode_blocking);
    int valido22 = simxGetObjectHandle(clientID, "youBot_ref#1", &Ox2, simx_opmode_blocking);
    int valido23 = simxGetObjectHandle(clientID, "youBot_positionTip", &youBot_Tip, simx_opmode_blocking);
    int valido24 = simxGetObjectHandle(clientID, "youBot_positionTip#1", &youBot_Tip2, simx_opmode_blocking);
    int valido25 = simxGetObjectHandle(clientID, "leftTarget", &leftTarget, simx_opmode_blocking);
    int valido26 = simxGetObjectHandle(clientID, "rightTarget", &rightTarget, simx_opmode_blocking);
    int valido27 = simxGetObjectHandle(clientID, "WorldVisual0", &WorldFrame2, simx_opmode_blocking);
    int valido28 = simxGetObjectHandle(clientID, "Force_sensor", &Force_sensor, simx_opmode_blocking);

    if(wheelsON == true){
        int valido6 = simxGetObjectHandle(clientID, "rollingJoint_rl", &wheel_rl, simx_opmode_blocking);
        int valido7 = simxGetObjectHandle(clientID, "rollingJoint_rr", &wheel_rr, simx_opmode_blocking);
        int valido8 = simxGetObjectHandle(clientID, "rollingJoint_fl", &wheel_fl, simx_opmode_blocking);
        int valido81 = simxGetObjectHandle(clientID, "rollingJoint_fr", &wheel_fr, simx_opmode_blocking);

        int valido14 = simxGetObjectHandle(clientID, "rollingJoint_rl#1", &wheel_rl2, simx_opmode_blocking);
        int valido15 = simxGetObjectHandle(clientID, "rollingJoint_rr#1", &wheel_rr2, simx_opmode_blocking);
        int valido16 = simxGetObjectHandle(clientID, "rollingJoint_fl#1", &wheel_fl2, simx_opmode_blocking);
        int valido18 = simxGetObjectHandle(clientID, "rollingJoint_fr#1", &wheel_fr2, simx_opmode_blocking);

        if (valido1 != 0 || valido2 != 0 || valido3 != 0 || valido4 != 0 || valido5 != 0 || valido6 != 0 || valido7 != 0  || valido8 != 0 || valido9 != 0 || valido10 != 0 || valido11 != 0 || valido12 != 0 || valido13 != 0 || valido14 != 0 || valido15 != 0 || valido16 != 0 || valido17 != 0 || valido18 != 0 || valido81 != 0 || valido19 != 0 ||  valido21 != 0 || valido22 != 0 || valido23 != 0 || valido24 != 0 || valido25 != 0 || valido26 != 0|| valido27 != 0|| valido28 != 0)
        {
            std::cout << "ERROR: Unable to connect with V-rep \n";
            std::cout << "Possible reason: One of the Bodies not detected: wheels ON Case \n";
            simxFinish(clientID);
            exit(1);
        }

    }
    if(wheelsON == false){
        int valido6 = simxGetObjectHandle(clientID, "World_X_Joint", &worldx, simx_opmode_blocking);
        int valido7 = simxGetObjectHandle(clientID, "World_Y_Joint", &worldy, simx_opmode_blocking);
        int valido8 = simxGetObjectHandle(clientID, "World_Th_Joint", &baseangle, simx_opmode_blocking);

        int valido14 = simxGetObjectHandle(clientID, "World_X_Joint0", &worldx2, simx_opmode_blocking);
        int valido15 = simxGetObjectHandle(clientID, "World_Y_Joint0", &worldy2, simx_opmode_blocking);
        int valido16 = simxGetObjectHandle(clientID, "World_Th_Joint0", &baseangle2, simx_opmode_blocking);

        if (valido1 != 0 || valido2 != 0 || valido3 != 0 || valido4 != 0 || valido5 != 0 || valido6 != 0 || valido7 != 0  || valido8 != 0 || valido9 != 0 || valido10 != 0 || valido11 != 0 || valido12 != 0 || valido13 != 0 || valido14 != 0 || valido15 != 0 || valido16 != 0 || valido17 != 0 || valido19 != 0 ||  valido21 != 0 || valido22 != 0 || valido23 != 0 || valido24 != 0 || valido25 != 0 || valido26 != 0|| valido27 != 0)
        {
            std::cout << "ERROR: Unable to connect with V-rep \n";
            std::cout << "Possible reason: One of the Bodies not detected \n";
            simxFinish(clientID);
            exit(1);
        }
    }


    simxGetObjectPosition(clientID,baseX,-1,&prismaticos[0],simx_opmode_streaming);
    simxGetObjectPosition(clientID,baseX2,WorldFrame2,&prismaticos2[0],simx_opmode_streaming);
    simxGetObjectPosition(clientID,youBot_Tip,-1,&tip[0],simx_opmode_streaming);
    simxGetObjectPosition(clientID,youBot_Tip2,WorldFrame2,&tip2[0],simx_opmode_streaming);
    simxGetObjectPosition(clientID,leftTarget,-1,&desired[0],simx_opmode_streaming);
    simxGetObjectPosition(clientID,rightTarget,WorldFrame2,&desired[0],simx_opmode_streaming);
    simxGetObjectOrientation(clientID, Ox,-1, &baseRot[0] ,simx_opmode_streaming);
    simxGetObjectOrientation(clientID, Ox2,WorldFrame2, &baseRot2[0] ,simx_opmode_streaming);


    simxReadForceSensor(clientID, Force_sensor,&Fsensor_state[0],&force[0], &torque[0],simx_opmode_streaming);


    // Set the initial configuration to Vrep
    if(wheelsON == true){
    simxSetJointTargetVelocity(clientID,wheel_fl, 0,simx_opmode_streaming);
    simxSetJointTargetVelocity(clientID,wheel_rl, 0,simx_opmode_streaming);
    simxSetJointTargetVelocity(clientID,wheel_rr, 0,simx_opmode_streaming);
    simxSetJointTargetVelocity(clientID,wheel_fr, 0,simx_opmode_streaming);
    simxSetJointTargetVelocity(clientID,wheel_fl2,0,simx_opmode_streaming);
    simxSetJointTargetVelocity(clientID,wheel_rl2,0,simx_opmode_streaming);
    simxSetJointTargetVelocity(clientID,wheel_rr2,0,simx_opmode_streaming);
    simxSetJointTargetVelocity(clientID,wheel_fr2,0,simx_opmode_streaming);
    }

    simxSetJointPosition(clientID, joint1, Q(3), simx_opmode_streaming);
    simxSetJointPosition(clientID, joint2, Q(4), simx_opmode_streaming);
    simxSetJointPosition(clientID, joint3, Q(5), simx_opmode_streaming);
    simxSetJointPosition(clientID, joint4, Q(6), simx_opmode_streaming);
    simxSetJointPosition(clientID, joint5, Q(7), simx_opmode_streaming);
    simxSetJointPosition(clientID, joint1_2, Q2(3), simx_opmode_streaming);
    simxSetJointPosition(clientID, joint2_2, Q2(4), simx_opmode_streaming);
    simxSetJointPosition(clientID, joint3_2, Q2(5), simx_opmode_streaming);
    simxSetJointPosition(clientID, joint4_2, Q2(6), simx_opmode_streaming);
    simxSetJointPosition(clientID, joint5_2, Q2(7), simx_opmode_streaming);
    for(int j=0; j<=300000; j++){
         simxGetObjectPosition(clientID,baseX,-1,&prismaticos[0],simx_opmode_buffer);
         simxGetObjectPosition(clientID,baseX2,WorldFrame2,&prismaticos2[0],simx_opmode_buffer);
         simxGetObjectPosition(clientID,youBot_Tip,-1,&tip[0],simx_opmode_buffer);
         simxGetObjectPosition(clientID,youBot_Tip2,WorldFrame2,&tip2[0],simx_opmode_buffer);
         simxGetObjectPosition(clientID,leftTarget,-1,&desired[0],simx_opmode_buffer);
         simxGetObjectPosition(clientID,rightTarget,WorldFrame2,&desired2[0],simx_opmode_buffer);
         simxGetObjectOrientation(clientID, Ox,-1, &baseRot[0] ,simx_opmode_buffer);
         simxGetObjectOrientation(clientID, Ox2,WorldFrame2, &baseRot2[0] ,simx_opmode_buffer);
         simxReadForceSensor(clientID, Force_sensor,&Fsensor_state[0],&force[0], &torque[0],simx_opmode_buffer);
         std::cout<<"loading positions ..."<<j<<std::endl;
     }

     resp << prismaticos[1], prismaticos[0], baseRot[2], prismaticos2[1], prismaticos2[0], baseRot2[2],
             tip[0], tip[1], tip[2], tip2[0], tip2[1], tip2[2], desired[0], desired[1], desired[2],
             desired2[0], desired2[1], desired2[2],force[0],force[1],force[2],torque[0],torque[1],torque[2];

}

VectorXd Simulador::connect(VectorXd Q,VectorXd Q2,VectorXd QDot,VectorXd Q2Dot, bool wheelsON){

     // Set the configuration to Vrep
     if(wheelsON == true){
     simxSetJointTargetVelocity(clientID,wheel_fl,(-QDot(1) +QDot(0) )*Kwh+QDot(2)*Krot,simx_opmode_streaming);
     simxSetJointTargetVelocity(clientID,wheel_rl,(-QDot(1) -QDot(0) )*Kwh+QDot(2)*Krot,simx_opmode_streaming);
     simxSetJointTargetVelocity(clientID,wheel_rr,(-QDot(1) +QDot(0) )*Kwh-QDot(2)*Krot,simx_opmode_streaming);
     simxSetJointTargetVelocity(clientID,wheel_fr,(-QDot(1) -QDot(0) )*Kwh-QDot(2)*Krot,simx_opmode_streaming);
     simxSetJointTargetVelocity(clientID,wheel_fl2,(-Q2Dot(1) +Q2Dot(0) )*Kwh+Q2Dot(2)*Krot,simx_opmode_oneshot);
     simxSetJointTargetVelocity(clientID,wheel_rl2,(-Q2Dot(1) -Q2Dot(0) )*Kwh+Q2Dot(2)*Krot,simx_opmode_oneshot);
     simxSetJointTargetVelocity(clientID,wheel_rr2,(-Q2Dot(1) +Q2Dot(0) )*Kwh-Q2Dot(2)*Krot,simx_opmode_oneshot);
     simxSetJointTargetVelocity(clientID,wheel_fr2,(-Q2Dot(1) -Q2Dot(0) )*Kwh-Q2Dot(2)*Krot,simx_opmode_oneshot);
     }
     if(wheelsON == false){
         simxSetJointPosition(clientID, worldx, Q(0), simx_opmode_streaming);
         simxSetJointPosition(clientID, worldy, Q(1), simx_opmode_streaming);
         simxSetJointPosition(clientID, baseangle, Q(2), simx_opmode_streaming);
         simxSetJointPosition(clientID, worldx2,  Q(0), simx_opmode_streaming);
         simxSetJointPosition(clientID, worldy2, Q(1), simx_opmode_streaming);
         simxSetJointPosition(clientID, baseangle2, Q2(2), simx_opmode_streaming);
     }

     simxSetJointPosition(clientID, joint1, Q(3), simx_opmode_streaming);
     simxSetJointPosition(clientID, joint2, Q(4), simx_opmode_streaming);
     simxSetJointPosition(clientID, joint3, Q(5), simx_opmode_streaming);
     simxSetJointPosition(clientID, joint4, Q(6), simx_opmode_streaming);
     simxSetJointPosition(clientID, joint5, Q(7), simx_opmode_streaming);
     simxSetJointPosition(clientID, joint1_2, Q2(3), simx_opmode_streaming);
     simxSetJointPosition(clientID, joint2_2, Q2(4), simx_opmode_streaming);
     simxSetJointPosition(clientID, joint3_2, Q2(5), simx_opmode_streaming);
     simxSetJointPosition(clientID, joint4_2, Q2(6), simx_opmode_streaming);
     simxSetJointPosition(clientID, joint5_2, Q2(7), simx_opmode_streaming);

     simxGetObjectPosition(clientID,baseX,-1,&prismaticos[0],simx_opmode_buffer);
     simxGetObjectPosition(clientID,baseX2,WorldFrame2,&prismaticos2[0],simx_opmode_buffer);
     simxGetObjectPosition(clientID,youBot_Tip,-1,&tip[0],simx_opmode_buffer);
     simxGetObjectPosition(clientID,youBot_Tip2,WorldFrame2,&tip2[0],simx_opmode_buffer);
     simxGetObjectPosition(clientID,leftTarget,-1,&desired[0],simx_opmode_buffer);
     simxGetObjectPosition(clientID,rightTarget,WorldFrame2,&desired2[0],simx_opmode_buffer);
     simxGetObjectOrientation(clientID, Ox,-1, &baseRot[0] ,simx_opmode_buffer);
     simxGetObjectOrientation(clientID, Ox2,WorldFrame2, &baseRot2[0] ,simx_opmode_buffer);
     simxReadForceSensor(clientID, Force_sensor,&Fsensor_state[0],&force[0], &torque[0],simx_opmode_buffer);

     resp << prismaticos[1], prismaticos[0], baseRot[2], prismaticos2[1], prismaticos2[0], baseRot2[2],
             tip[0], tip[1], tip[2], tip2[0], tip2[1], tip2[2], desired[0], desired[1], desired[2],
             desired2[0], desired2[1], desired2[2],force[0],force[1],force[2],torque[0],torque[1],torque[2];
    /*
     std::cout<< " Fuerza X:"<< force[0] <<std::endl;
     std::cout<< " Fuerza Y:"<< force[1] <<std::endl;
     std::cout<< " Fuerza Z:"<< force[2] <<std::endl;
     std::cout<<"\n"<<" Torque X:"<< torque[0] <<std::endl;
     std::cout<< " torque Y:"<< torque[1] <<std::endl;
     std::cout<< " torque Z:"<< torque[2] <<std::endl;
    */
     return resp;
 }

}




