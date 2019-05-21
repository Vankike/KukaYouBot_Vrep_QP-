
#include <vrepInit2.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
//! Funciona con la escena force_control_experiment.tt
extern "C" {
#include "extApi.h"
    /*#include "extApiCustom.h" if you wanna use custom remote API functions! */
}
using namespace Eigen;

namespace VREP{

Simulador::Simulador(VectorXd Q, bool wheelsON){

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


     valido[0] = simxGetObjectHandle(clientID, "Joint1", &joint1, simx_opmode_blocking);
     valido[1] = simxGetObjectHandle(clientID, "Joint2", &joint2, simx_opmode_blocking);
     valido[2] = simxGetObjectHandle(clientID, "Joint3", &joint3, simx_opmode_blocking);
     valido[3] = simxGetObjectHandle(clientID, "Joint4", &joint4, simx_opmode_blocking);
     valido[4] = simxGetObjectHandle(clientID, "Joint5", &joint5, simx_opmode_blocking);


     valido[5] = simxGetObjectHandle(clientID, "youBot", &baseX, simx_opmode_blocking);
     valido[6] = simxGetObjectHandle(clientID, "youBot_ref", &Ox, simx_opmode_blocking);
     valido[7] = simxGetObjectHandle(clientID, "youBot_positionTip", &youBot_Tip, simx_opmode_blocking);
     valido[8] = simxGetObjectHandle(clientID, "Pc", &Pc, simx_opmode_blocking);
     valido[9] = simxGetObjectHandle(clientID, "Pk", &Pk, simx_opmode_blocking);
     valido[10] = simxGetObjectHandle(clientID, "Pj", &Pj, simx_opmode_blocking);
     valido[11] = simxGetObjectHandle(clientID, "Force_sensor", &Force_sensor, simx_opmode_blocking);

    if(wheelsON == true){
         valido[12] = simxGetObjectHandle(clientID, "rollingJoint_rl", &wheel_rl, simx_opmode_blocking);
         valido[13] = simxGetObjectHandle(clientID, "rollingJoint_rr", &wheel_rr, simx_opmode_blocking);
         valido[14] = simxGetObjectHandle(clientID, "rollingJoint_fl", &wheel_fl, simx_opmode_blocking);
         valido[15] = simxGetObjectHandle(clientID, "rollingJoint_fr", &wheel_fr, simx_opmode_blocking);

    for(int I = 0 ; I <= 15; I++){
        if (valido[I] != 0 )
        {
            std::cout << "ERROR: Unable to connect with V-rep \n";
            std::cout << "Possible reason: One of the Bodies not detected: wheels ON Case \n";
            simxFinish(clientID);
            exit(1);
        }
    }

    }
    if(wheelsON == false){
         valido[12] = simxGetObjectHandle(clientID, "World_X_Joint", &worldx, simx_opmode_blocking);
         valido[13] = simxGetObjectHandle(clientID, "World_Y_Joint", &worldy, simx_opmode_blocking);
         valido[14] = simxGetObjectHandle(clientID, "World_Th_Joint", &baseangle, simx_opmode_blocking);
        for(int I = 0 ; I <= 14; I++){
            if (valido[I] != I )
            {
                std::cout << "ERROR: Unable to connect with V-rep \n";
                std::cout << "Possible reason: One of the Bodies not detected: wheels ON Case \n";
                simxFinish(clientID);
                exit(1);
            }
        }
    }


    simxGetObjectPosition(clientID,baseX,-1,&prismaticos[0],simx_opmode_streaming);  
    simxGetObjectPosition(clientID,youBot_Tip,-1,&tip[0],simx_opmode_streaming);
    simxGetObjectPosition(clientID,Pc,-1,&desired[0],simx_opmode_streaming);
    simxGetObjectPosition(clientID,Pk,-1,&PuntoK[0],simx_opmode_streaming);
    simxGetObjectPosition(clientID,Pj,-1,&PuntoJ[0],simx_opmode_streaming);
    simxGetObjectOrientation(clientID, Ox,-1, &baseRot[0] ,simx_opmode_streaming);
    simxReadForceSensor(clientID, Force_sensor,&Fsensor_state[0],&force[0], &torque[0],simx_opmode_streaming);


    // Set the initial configuration to Vrep
    if(wheelsON == true){
    simxSetJointTargetVelocity(clientID,wheel_fl, 0,simx_opmode_streaming);
    simxSetJointTargetVelocity(clientID,wheel_rl, 0,simx_opmode_streaming);
    simxSetJointTargetVelocity(clientID,wheel_rr, 0,simx_opmode_streaming);
    simxSetJointTargetVelocity(clientID,wheel_fr, 0,simx_opmode_streaming);
    }

    simxSetJointPosition(clientID, joint1, Q(3), simx_opmode_streaming);
    simxSetJointPosition(clientID, joint2, Q(4), simx_opmode_streaming);
    simxSetJointPosition(clientID, joint3, Q(5), simx_opmode_streaming);
    simxSetJointPosition(clientID, joint4, Q(6), simx_opmode_streaming);
    simxSetJointPosition(clientID, joint5, Q(7), simx_opmode_streaming);

    for(int j=0; j<=300000; j++){
         simxGetObjectPosition(clientID,baseX,-1,&prismaticos[0],simx_opmode_buffer);       
         simxGetObjectPosition(clientID,youBot_Tip,-1,&tip[0],simx_opmode_buffer);        
         simxGetObjectPosition(clientID,Pc,-1,&desired[0],simx_opmode_buffer);
         simxGetObjectPosition(clientID,Pk,-1,&PuntoK[0],simx_opmode_buffer);
         simxGetObjectPosition(clientID,Pj,-1,&PuntoJ[0],simx_opmode_buffer);
         simxGetObjectOrientation(clientID, Ox,-1, &baseRot[0] ,simx_opmode_buffer);
         simxReadForceSensor(clientID, Force_sensor,&Fsensor_state[0],&force[0], &torque[0],simx_opmode_buffer);
         std::cout<<"loading positions ..."<<j<<std::endl;
     }

     resp << prismaticos[1], prismaticos[0], baseRot[2],
             tip[0], tip[1], tip[2], desired[0], desired[1], desired[2],
            force[0],force[1],force[2],torque[0],torque[1],torque[2],
             PuntoK[0],PuntoK[1],PuntoK[2],PuntoJ[0],PuntoJ[1],PuntoJ[2];

}

VectorXd Simulador::connect(VectorXd Q,VectorXd QDot, bool wheelsON){

     // Set the configuration to Vrep
     if(wheelsON == true){
     simxSetJointTargetVelocity(clientID,wheel_fl,(-QDot(1) +QDot(0) )*Kwh+QDot(2)*Krot,simx_opmode_streaming);
     simxSetJointTargetVelocity(clientID,wheel_rl,(-QDot(1) -QDot(0) )*Kwh+QDot(2)*Krot,simx_opmode_streaming);
     simxSetJointTargetVelocity(clientID,wheel_rr,(-QDot(1) +QDot(0) )*Kwh-QDot(2)*Krot,simx_opmode_streaming);
     simxSetJointTargetVelocity(clientID,wheel_fr,(-QDot(1) -QDot(0) )*Kwh-QDot(2)*Krot,simx_opmode_streaming);
     }
     if(wheelsON == false){
         simxSetJointPosition(clientID, worldx, Q(0), simx_opmode_streaming);
         simxSetJointPosition(clientID, worldy, Q(1), simx_opmode_streaming);
         simxSetJointPosition(clientID, baseangle, Q(2), simx_opmode_streaming);       
     }

     simxSetJointPosition(clientID, joint1, Q(3), simx_opmode_streaming);
     simxSetJointPosition(clientID, joint2, Q(4), simx_opmode_streaming);
     simxSetJointPosition(clientID, joint3, Q(5), simx_opmode_streaming);
     simxSetJointPosition(clientID, joint4, Q(6), simx_opmode_streaming);
     simxSetJointPosition(clientID, joint5, Q(7), simx_opmode_streaming);

     simxGetObjectPosition(clientID,baseX,-1,&prismaticos[0],simx_opmode_buffer);    
     simxGetObjectPosition(clientID,youBot_Tip,-1,&tip[0],simx_opmode_buffer);    
     simxGetObjectPosition(clientID,Pc,-1,&desired[0],simx_opmode_buffer);
     simxGetObjectPosition(clientID,Pk,-1,&PuntoK[0],simx_opmode_buffer);
     simxGetObjectPosition(clientID,Pj,-1,&PuntoJ[0],simx_opmode_buffer);
     simxGetObjectOrientation(clientID, Ox,-1, &baseRot[0] ,simx_opmode_buffer);
     simxReadForceSensor(clientID, Force_sensor,&Fsensor_state[0],&force[0], &torque[0],simx_opmode_buffer);

     resp << prismaticos[1], prismaticos[0], baseRot[2],
             tip[0], tip[1], tip[2], desired[0], desired[1], desired[2],
            force[0],force[1],force[2],torque[0],torque[1],torque[2],
             PuntoK[0],PuntoK[1],PuntoK[2],PuntoJ[0],PuntoJ[1],PuntoJ[2];

     return resp;
 }

}




