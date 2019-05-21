// Esta versión fue creada para trabajar con vrep y dos robots kukayoubot
// esta orientado a objetos


// SOLO FUNCIONó para probar el envio de datos a vrep, a los joints de los brazos sin controlar la base. 

// cada kuka resuelve su dinámica y la tarea de posición y orientación se resuleve mediante un unico QPSOLVER (QPOASES)
// con las matrices apiladas en diagonal y los vectores en forma vertical. conforme el método usado por Karim B. en multirobot force control .
// al 04/abril/2019
// BY L. HERNANDEZ-SANCHEZ


#include <qpOASES.hpp>
#include <iostream>
#include <rbdl/rbdl.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <stdio.h>
#include<stdlib.h>

#include <time.h>

extern "C" {
#include "extApi.h"
    /*#include "extApiCustom.h" if you wanna use custom remote API functions! */
}

using namespace RigidBodyDynamics;
using namespace Eigen;
USING_NAMESPACE_QPOASES

int SavetoFile(FILE *dataFile,FILE *dataFile2,FILE *dataFile3,FILE *dataFile4,FILE *dataFile5,
               VectorXd QDDot,VectorXd QDot, VectorXd Q, VectorXd err,VectorXd FT){
    // save angles  to a file .txt
    fprintf(dataFile,"%.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f", Q(0),Q(1),Q(2),Q(3),Q(4),Q(5),Q(6),Q(7));
    fprintf(dataFile,"\n");
    // save vel.  to a file .txt
    fprintf(dataFile2,"%.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f",QDot(0),QDot(1),QDot(2),QDot(3), QDot(4),QDot(5),QDot(6),QDot(7));
    fprintf(dataFile2,"\n");
    // save acc.  to a file .txt
    fprintf(dataFile3,"%.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f",QDDot(0),QDDot(1),QDDot(2),QDDot(3), QDDot(4),QDDot(5),QDDot(6),QDDot(7));
    fprintf(dataFile3,"\n");
    // save errors to a file .txt
    fprintf(dataFile4,"%.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f",err(0),err(1),err(2),err(3), err(4),err(5));
    fprintf(dataFile4,"\n");
    // save generalized forces to a file .txt
    fprintf(dataFile5,"%.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f",FT(0),FT(1),FT(2),FT(3), FT(4),FT(5),FT(6),FT(7));
    fprintf(dataFile5,"\n");
    return 0;
}

int msleep(unsigned long milisec)
{
    struct timespec req={0};
    time_t sec=(int)(milisec/1000);
    milisec=milisec-(sec*1000);
    req.tv_sec=sec;
    req.tv_nsec=milisec*1000000L;
    while(nanosleep(&req,&req)==-1)
        continue;
    return 1;
}


class KUKA{

 private:
    #define Dof     8
    #define DofTot  16
    #define nV      32
    #define cOns	44
    #define nV2     32
    #define cOns2	44
    bool update_kinematics, UpBnd, lwBnd ;
    double l1,l2,l3,l4,l5,l6,l7, t0, tEnd, delta,delta2, Kp, Kd, Num_Iter;
    int Finaltime, NN, RegSteps, RegSteps2, nWSR, nWSR2;
    unsigned int body_a_id, body_b_id, body_c_id, body_d_id, body_e_id,  body_f_id, body_g_id, body_h_id;
    Body body_a, body_b, body_c, body_d, body_e, body_f, body_g, body_h;
    Joint joint_a, joint_b, joint_c, joint_d, joint_e, joint_f, joint_g, joint_h;
    Vector3d r1, r2, r3, r4, r5, r6, r7, r8, CoM1, CoM2, CoM3, CoM4, CoM5, CoM6, CoM7, CoM8,
             JointAxisZ, dP, dPh, Z, w, dQ_13, e, eQ , o3, od3, pos, posD,
             e2, eQ2 , o3_2, od3_2, pos2, posD2;
    Vector4d Quat, dQuat, o4, od4,
             Quat2, dQuat2, o4_2, od4_2;

    VectorXd mass{Dof}, h{Dof}, qS{Dof},qI{Dof}, qM{Dof}, QS{Dof},QI{Dof},fr{Dof},D{Dof},Q{Dof}, QDot{Dof}, QDDot{Dof},
             FT{Dof},TrG{nV},TrG2{nV2},g{nV},g2{nV2},ub{nV},lb{nV},lbA{cOns},lbA2{cOns2},ubA{cOns},ubA2{cOns2}, Ax{cOns},
             xOpt1{nV},xOpt2{nV2}, POSE{7},err{6}, bA{6},
             Q2{Dof}, Q2Dot{Dof}, Q2DDot{Dof}, h2{Dof}, POSE2{7}, Q2S{Dof},Q2I{Dof}, err2{6}, bA2{6};
    Math::Matrix3d Ti1, Ti2, Ti3,Ti4,Ti5, Ti6, Ti7,Ti8,E1, E2, E3, E4, E5, E6, E7, E8;
    Matrix3d O, od3hat, eye3, zero3, dJthInv,
             O2, od3hat2;
    MatrixXd H{Dof,Dof}, Ja{6,Dof}, dJa{6,Dof}, Ag{6,6}, Jg{6, Dof}, dJg{6, Dof}, dJxInv{6, 6}, JxInv{6, 6}, Jeps{3,4},
             Jth{3,4}, JthInv{4,3}, Fricc{Dof,Dof}, eye8{8,8}, zeros68{6,Dof}, zeros8{8,8}, G{nV,nV},
             G2{nV2,nV2}, zeros19_3{19,3},
             H2{Dof,Dof},J2a{6,Dof}, dJ2a{6,Dof}, H_Total{DofTot,DofTot},Ja_Total{12,DofTot},eye16{16,16},zeros16{16,16};

    Eigen::Matrix< double,cOns ,nV ,RowMajor> A;
    Eigen::Matrix< double,cOns2,nV2,RowMajor> A2;
    real_t eps,eps2;
    Model* model;
    struct Positions{
            Vector3d o;
        } p[Dof];
    struct Twist{
            VectorXd u{6};
        } N[Dof];
    struct JacobiansG{
            MatrixXd G{6,Dof};
        } J[Dof];
    struct dynamic{
            VectorXd h{Dof},Pose{7};
            MatrixXd H{Dof,Dof},Ja{6,Dof},dJa{6,Dof};
        };
    qpOASES::Options myOptions, myOptions2;

  public:

    KUKA () { // Constructor. Parameters of the KUKAyouBot

        /*******************************************************************************
        * Initial Values
        *******************************************************************************/
        /*-1.38446700000000
        0.809505000000000
        2.91566200000000
        2.91566200000000
        1.48974800000000
        2.50101800000000
        0.196742000000000 ORIENTACION PROBLEMATICA PARA EXAMINAR
        0*/
        Q  <<  0,  0 ,  0 ,  0 , 0.2, -0.1, 0.02,  0;
        Q2  <<  0,  0 ,  0 ,  0 , 0.2, -0.1, 0.02,  0;

        fr <<  8,  8 , 0.1, 0.1, 0.3,  0.3, 0.2,  0.07;
        Kp = 0.2;
        Kd = 0.8;
        delta = 0.01;
        delta2 = 0.1;
        t0 = 0;
        tEnd = 30;

        posD << 0.65, -0.974, 0.55;
        posD2 << 0.65, -0.974, 0.55;
        //posD << 1.5, -2, 0.2;
        od4  << 0.7011 , 0, 0.7011, 0;
        od4_2  << 0.7011 , 0, 0.7011, 0;
        //od4  << 0.258819,0, 0.9659258, 0;
        Finaltime = tEnd;
        Num_Iter = (tEnd - t0)/delta;
        NN = Num_Iter;
        QDot.setZero();
        QDDot.setZero();
        Q2Dot.setZero();
        Q2DDot.setZero();
        h.setZero();
        FT.setZero();
        H.setZero();
        H2.setZero();
        h2.setZero();
        eye8.setIdentity();
        eye16.setIdentity();
        zeros68.setZero();
        zeros8.setZero();
        zeros16.setZero();
        Jg.setZero();
        dJg.setZero();
        zero3.setZero();
        Ag.setZero();
        eye3.setIdentity();
        zeros19_3.setZero();
        xOpt1.setZero();
        xOpt2.setZero();
        Fricc  = fr.asDiagonal();
        for (int i = 7; i >= 0; i--)  J[i].G.setZero();
        update_kinematics = true;

        /*******************************************************************************
        * Joint Limits
        *******************************************************************************/
        qS <<  5,  5,  M_PI,  2.932,  1.553,  2.53,  1.78,  2.89;
        qI << -5, -5, -M_PI, -2.932, -1.1170, -2.62, -1.78, -2.89;
        qM << 0.1, 0.1,   0.1,    0.4,    0.1,   0.1,   0.1,   0.1;

        /*******************************************************************************
        * QP  Parameters
        *******************************************************************************/

        nWSR = 300;
        eps= 100;
        RegSteps = 0;

        myOptions.printLevel = qpOASES::PL_LOW;
        myOptions.enableEqualities = qpOASES::BT_TRUE;
        myOptions.enableRegularisation = qpOASES::BT_TRUE;
        myOptions.numRegularisationSteps = RegSteps;
        myOptions.epsRegularisation = eps;

        TrG.setOnes();
        TrG(nV-1) = 0.9;
        g.setZero();
        G.setZero();


        nWSR2 =300;
        eps2= 100;
        RegSteps2 = 0;

        myOptions2.printLevel = qpOASES::PL_LOW;
        myOptions2.enableEqualities = qpOASES::BT_TRUE;
        myOptions2.enableRegularisation = qpOASES::BT_TRUE;
        myOptions2.numRegularisationSteps = RegSteps2;
        myOptions2.epsRegularisation = eps2;

        //myOptions2.setToDefault();

        TrG2.setOnes();
        TrG2 = TrG2*0.001;
        TrG2(nV2-1) = 1;
        TrG2(nV2-2) = 1;
        TrG2(nV2-3) = 1;
        g2.setZero();
        G2.setZero();

        /*******************************************************************************************************************
        * Model Parameters (Gravity, Link_Lengths, Action_Axis_Z, Masses, Centers_Of_Mass, X_T, Inertia_Tensioners, Bodies )
        ********************************************************************************************************************/

        model = new Model();
        model->gravity = Math::Vector3d (0.,0.,-9.81);
        l1 = 0.084; l2 = 0.143; l3 = 0.161; l4 = 0.155; l5 = 0.135; l6 = 0.05; l7 = 0.113; JointAxisZ << 0, 0, 1;
        CoM1 <<-0.033,  0,  0; CoM2 << 0,   0.033,   0; CoM3 << 0,    0,    0.033; CoM4 <<  0,       0,   -0.058;
        CoM5 << 0.113,  0,  0; CoM6 << 0.104,   0,   0; CoM7 << 0, -0.053,    0;   CoM8 <<  0,       0,    0.046;
        r1 <<  0, 0, 0; r2 <<-l1, 0, 0; r3 <<  0, 0, 0; r4 << l2,  0,l3;
        r5 <<  0, 0, 0; r6 << l4, 0, 0; r7 << l5, 0, 0; r8 <<-l6,-l7, 0;
        mass << 0,0,30,1.390, 1.318, 0.821, 0.769, 0.8;
        Ti1 << 0 , 0 ,  0, 0 , 0 ,  0, 0 , 0 ,  0  ;
        Ti2 << 0 , 0 ,  0, 0 , 0 ,  0, 0 , 0 ,  0  ;
        Ti3 << 0.705, 0, 0,0, 0.192, 0, 0, 0, 0.255;
        Ti4 << 0.00696 , 0  ,    0, 0   , 0.00768,           0,0       , 0      , 0.00676;
        Ti5 << 0.00228 , 0  ,    0, 0   , 0.0023 ,           0,0       , 0      , 0.000483;
        Ti6 << 0.00129 , 0  ,    0, 0   , 0.0013 ,           0,0       , 0      , 0.00025;
        Ti7 << 0.000692, 0  ,    0, 0   , 0.000489 ,         0,0       , 0      , 0.000425;
        Ti8 << 0.000124, 0  ,    0, 0   , 6.64e-005 ,        0,0       , 0      , 0.000161;
        E1 <<  0 , 0 ,-1,-1 , 0 , 0,0 , 1 , 0;
        E2 <<  0 , 0 , 1,-1 , 0 , 0,0 ,-1 , 0;
        E3 <<  0 , 0 , 1, 1 , 0 , 0,0 , 1 , 0;
        E4 <<  1 , 0 , 0, 0 ,-1 , 0,0 , 0 ,-1;
        E5 <<  0 , 0 ,-1, 1 , 0 , 0,0 ,-1 , 0;
        E6 <<  1 , 0 , 0, 0 , 1 , 0,0 , 0 , 1;
        E7 <<  0 , 1 , 0,-1 , 0 , 0,0 , 0 , 1;
        E8 <<  1 , 0 , 0, 0 , 0 , 1,0 , -1, 0;

        body_a = Body (mass(0), CoM1, Ti1 );    joint_a = Joint( JointTypePrismatic, JointAxisZ );
        body_a_id = model->AddBody(    0    , Math::SpatialTransform( E1, r1),  joint_a, body_a);
        body_b = Body (mass(1), CoM2,  Ti2 );   joint_b = Joint( JointTypePrismatic, JointAxisZ );
        body_b_id = model->AddBody(body_a_id, Math::SpatialTransform( E2, r2),  joint_b, body_b);
        body_c = Body (mass(2), CoM3,  Ti3 );   joint_c = Joint ( JointTypeRevolute, JointAxisZ );
        body_c_id = model->AddBody(body_b_id, Math::SpatialTransform( E3, r3) , joint_c, body_c);
        body_d = Body (mass(3), CoM4,  Ti4 );   joint_d = Joint ( JointTypeRevolute, JointAxisZ );
        body_d_id = model->AddBody(body_c_id, Math::SpatialTransform( E4, r4) , joint_d, body_d);
        body_e = Body (mass(4), CoM5,  Ti5 );   joint_e = Joint ( JointTypeRevolute, JointAxisZ );
        body_e_id = model->AddBody(body_d_id, Math::SpatialTransform( E5, r5) , joint_e, body_e);
        body_f = Body (mass(5), CoM6,  Ti6 );   joint_f = Joint ( JointTypeRevolute, JointAxisZ );
        body_f_id = model->AddBody(body_e_id, Math::SpatialTransform( E6, r6) , joint_f, body_f);
        body_g = Body (mass(6), CoM7,  Ti7 );   joint_g = Joint ( JointTypeRevolute, JointAxisZ );
        body_g_id = model->AddBody(body_f_id, Math::SpatialTransform( E7, r7) , joint_g, body_g);
        body_h = Body (mass(7), CoM8,  Ti8 );   joint_h = Joint ( JointTypeRevolute, JointAxisZ );
        body_h_id = model->AddBody(body_g_id, Math::SpatialTransform( E8, r8) , joint_h, body_h);

        }

    dynamic HandC(VectorXd Q,VectorXd QDot){//Return Inertia Matrix, Nonlinear terms, Analytic Jacobian and its derivative and the Pose

            dynamic results;
            // -- CRBA From RBDL -- //
            RigidBodyDynamics::CompositeRigidBodyAlgorithm (*model, Q, results.H, update_kinematics);

            // -- RNEA From RBDL -- //
            RigidBodyDynamics::NonlinearEffects (*model, Q, QDot, results.h);
            // -- For to obtain all the Geometric Jacobians by recursivity-- //
            for (int i = 7; i >= 0; i--)
            {
                p[i].o = CalcBodyToBaseCoordinates( *model, Q, i+1, Math::Vector3d(0., 0., 0.), update_kinematics );
                O = CalcBodyWorldOrientation 	( 	*model, Q, i+1 ,update_kinematics).transpose();

                Z << O(0, 2), O(1, 2), O(2, 2);
                if (i <= 1)  J[7].G.col(i) << Z, 0, 0, 0;
                else         J[7].G.col(i) << Z.cross(p[7].o - p[i].o), Z;

                if (i <=6){
                    if (i <= 1)  J[6].G.col(i) << Z, 0, 0, 0;
                    else         J[6].G.col(i) << Z.cross(p[6].o - p[i].o), Z;
                }
                if (i <=5){
                    if (i <= 1)  J[5].G.col(i) << Z, 0, 0, 0;
                    else         J[5].G.col(i) << Z.cross(p[5].o - p[i].o), Z;
                }
                if (i <=4){
                    if (i <= 1)  J[4].G.col(i) << Z, 0, 0, 0;
                    else         J[4].G.col(i) << Z.cross(p[4].o - p[i].o), Z;
                }
                if (i <=3){
                    if (i <= 1)  J[3].G.col(i) << Z, 0, 0, 0;
                    else         J[3].G.col(i) << Z.cross(p[3].o - p[i].o), Z;
                }
                if (i <=2){
                    if (i <= 1)  J[2].G.col(i) << Z, 0, 0, 0;
                    else         J[2].G.col(i) << Z.cross(p[2].o - p[i].o), Z;
                }
                if (i <=1){
                    if (i <= 1)  J[1].G.col(i) << Z, 0, 0, 0;
                    else         J[1].G.col(i) << Z.cross(p[1].o - p[i].o), Z;
                }
                if (i ==0){
                    if (i <= 1)  J[0].G.col(i) << Z, 0, 0, 0;
                    else         J[0].G.col(i) << Z.cross(p[0].o - p[i].o), Z;
                }
            }
            // -- For to obtain the Twist of all bodies -- //
            for (int i = 7; i >= 0; i--)  N[i].u = J[i].G*QDot;
            // -- Lineal Velocity of Body h (or Last Body) -- //
            dPh = N[7].u.block(0,0,3,1);
            // -- Geometric Jacobian on the End Effector (E.E)-- //
            Jg = J[7].G;
            // -- For to obtain the Time derivative (T.D.) of E.E. Geometric Jacobian by means of Ag matrix -- //
            for (int i = 0; i <= 7; i++)
            {
              dP = N[i].u.block(0,0,3,1);
              w = N[i].u.block(3,0,3,1);
              Ag << skew(w), -skew((dPh-dP)-skew(w)*(p[7].o - p[i].o)),
                      zero3,   skew(w);
              dJg.col(i) = Ag *Jg.col(i);
            }
            // -- Function RBDL to calculate the Rotation Matrix of Final body -- //
            O = CalcBodyWorldOrientation(*model, Q, model->previously_added_body_id,update_kinematics).transpose();
            // -- Obtain the Unit Quaternion   -- //
            Quat = Quater(O);
            // -- Epsilon Jacobian  with quaternions -- //
            Jeps << -Quat(1), Quat(0), -Quat(3), Quat(2),
                    -Quat(2), Quat(3),  Quat(0), -Quat(1),
                    -Quat(3),-Quat(2),  Quat(1), Quat(0);
            // -- Theta Jacobian with quaternions -- //
            Jth = 2*Jeps;
            // -- Inverse of the theta Jacobian -- //
            JthInv = 0.5*(Jeps.transpose());
            // -- T.D. of the Unit Quaternion -- //
            dQuat = JthInv*w;
            // -- Vector part  -- //
            dQ_13 << dQuat(1),dQuat(2),dQuat(3);
            // -- Inverse of the T.D. Theta Jacobian -- //
            dJthInv << dQuat(0)*eye3-skew(dQ_13);
            // -- Inverse Jx  -- //
            JxInv <<   eye3, zero3,
                      zero3, JthInv.block(1,0,3,3);
            // -- T.D. Jx  -- //
            dJxInv <<   zero3, zero3,
                     zero3, dJthInv;
            // -- Analytic Jacobian  -- //
            //Ja = (JxInv*Jg).transpose();
            results.Ja = JxInv*Jg;
            // -- T.D. of Analytic Jacobian  -- //
            results.dJa = JxInv*dJg + dJxInv*Jg;
            // -- Position & Orientation of the E.E.  -- //
            results.Pose <<  p[7].o,Quat;
            // -- Return Inertial Matrix (H), Nonlinear Terms(h), Ja, dJa and Pose in a single Matrix  -- //
            return results;
        }

    Vector4d Quater(Matrix3d R){ //std::cout<<"quaternion scalar"<<P.w()<<"Vector"<<std::endl<<P.x()<< std::endl;
            Quaterniond P(R);    Vector4d QuatEig;
            QuatEig(0) = P.w();     QuatEig(1) = P.x();     QuatEig(2) = P.y();     QuatEig(3) = P.z();
            return QuatEig;
        }

    Matrix3d skew(MatrixXd v) { // Skewsymmetric Matrix Form
            Matrix3d S;
            S <<     0, -v(2),  v(1),
                  v(2),     0, -v(0),
                 -v(1),  v(0),     0;
            return S;
        }

    void QPsolver(int clientID)
        {
            // Open a .txt file to save data.
            FILE *dataFile = fopen("hqp_poses.txt","w");
            FILE *dataFile2= fopen("hqp_vel.txt","w");
            FILE *dataFile3= fopen("hqp_acc.txt","w");
            FILE *dataFile4= fopen("hqp_e.txt","w");
            FILE *dataFile5= fopen("hqp_FT.txt","w");
            // structure dynamic
            dynamic ans;
            dynamic ans2;

            /// Configure the connection with Vrep
            int worldx, worldy, baseangle, joint1, joint2, joint3, joint4, joint5, worldx2,
                worldy2, baseangle2, joint1_2, joint2_2, joint3_2, joint4_2, joint5_2;

            //Define the port connection
            int valido1 = simxGetObjectHandle(clientID, "Joint1", &joint1, simx_opmode_blocking);
            int valido2 = simxGetObjectHandle(clientID, "Joint2", &joint2, simx_opmode_blocking);
            int valido3 = simxGetObjectHandle(clientID, "Joint3", &joint3, simx_opmode_blocking);
            int valido4 = simxGetObjectHandle(clientID, "Joint4", &joint4, simx_opmode_blocking);
            int valido5 = simxGetObjectHandle(clientID, "Joint5", &joint5, simx_opmode_blocking);
            //int valido6 = simxGetObjectHandle(clientID, "World_X_Joint", &worldx, simx_opmode_blocking);
            //int valido7 = simxGetObjectHandle(clientID, "World_Y_Joint", &worldy, simx_opmode_blocking);
            //int valido8 = simxGetObjectHandle(clientID, "World_Th_Joint", &baseangle, simx_opmode_blocking);
            int valido9 = simxGetObjectHandle(clientID, "Joint1#1", &joint1_2, simx_opmode_blocking);
            int valido10 = simxGetObjectHandle(clientID, "Joint2#1", &joint2_2, simx_opmode_blocking);
            int valido11 = simxGetObjectHandle(clientID, "Joint3#1", &joint3_2, simx_opmode_blocking);
            int valido12 = simxGetObjectHandle(clientID, "Joint4#1", &joint4_2, simx_opmode_blocking);
            int valido13 = simxGetObjectHandle(clientID, "Joint5#1", &joint5_2, simx_opmode_blocking);
            //int valido14 = simxGetObjectHandle(clientID, "World_X_Joint#0", &worldx2, simx_opmode_blocking);
            //int valido15 = simxGetObjectHandle(clientID, "World_Y_Joint#0", &worldy2, simx_opmode_blocking);
            //int valido16 = simxGetObjectHandle(clientID, "World_Th_Joint#0", &baseangle2, simx_opmode_blocking);

            //If the connection is ok then start the simulation.
            if (clientID == -1)
            {
                std::cout << "ERROR: Unable to connect with V-rep \n";
                std::cout << "Possible reason: First click on play in vrep \n";

                simxFinish(clientID);
                exit(1);
            }
            if (valido1 != 0 || valido2 != 0 || valido3 != 0 || valido4 != 0 || valido5 != 0  || valido9 != 0 || valido10 != 0 || valido11 != 0 || valido12 != 0 || valido13 != 0 )
            {
                std::cout << "ERROR: Unable to connect with V-rep \n";
                simxFinish(clientID);

                exit(1);
            }

            // Set the initial configuration to Vrep
            //simxSetJointPosition(clientID, worldx, Q(0), simx_opmode_streaming);
            //simxSetJointPosition(clientID, worldy, Q(1), simx_opmode_streaming);
            //simxSetJointPosition(clientID, baseangle,Q(2), simx_opmode_streaming);
            simxSetJointPosition(clientID, joint1, Q(3), simx_opmode_streaming);
            simxSetJointPosition(clientID, joint2, Q(4), simx_opmode_streaming);
            simxSetJointPosition(clientID, joint3, Q(5), simx_opmode_streaming);
            simxSetJointPosition(clientID, joint4, Q(6), simx_opmode_streaming);
            simxSetJointPosition(clientID, joint5, Q(7), simx_opmode_streaming);
            //simxSetJointPosition(clientID, worldx2, Q2(0), simx_opmode_streaming);
            //simxSetJointPosition(clientID, worldy2, Q2(1), simx_opmode_streaming);
            //simxSetJointPosition(clientID, baseangle2,Q2(2), simx_opmode_streaming);
            simxSetJointPosition(clientID, joint1_2, Q2(3), simx_opmode_streaming);
            simxSetJointPosition(clientID, joint2_2, Q2(4), simx_opmode_streaming);
            simxSetJointPosition(clientID, joint3_2, Q2(5), simx_opmode_streaming);
            simxSetJointPosition(clientID, joint4_2, Q2(6), simx_opmode_streaming);
            simxSetJointPosition(clientID, joint5_2, Q2(7), simx_opmode_streaming);

            for (int i=1;i<NN;i++){

                ans = HandC(Q,QDot);
                ans2 = HandC(Q2,Q2Dot);

                H = ans.H;                 
                h = ans.h;
                H2 = ans2.H;
                h2 = ans2.h;

                Ja = ans.Ja;
                dJa = ans.dJa;
                POSE = ans.Pose;

                J2a = ans2.Ja;
                dJ2a = ans2.dJa;
                POSE2 = ans2.Pose;


                Ja_Total <<    Ja   , zeros68,
                            zeros68 ,   J2a  ;


                pos = POSE.block(0,0,3,1);
                o4  = POSE.block(3,0,4,1);
                o3 << o4[1],o4[2],o4[3];
                od3 << od4[1],od4[2],od4[3];
                od3hat << 0   , -od4[3],  od4[2],
                        od4[3],    0   , -od4[1],
                       -od4[2],  od4[1],     0;
                QS = (2*(qS-Q-delta2*QDot))/(delta2*delta2);
                QI = (2*(qI-Q-delta2*QDot))/(delta2*delta2);

                pos2 = POSE2.block(0,0,3,1);
                o4_2  = POSE2.block(3,0,4,1);
                o3_2 << o4_2[1],o4_2[2],o4_2[3];
                od3_2 << od4_2[1],od4_2[2],od4_2[3];
                od3hat2 << 0     , -od4_2[3],  od4_2[2],
                        od4_2[3],      0   , -od4_2[1],
                       -od4_2[2],  od4_2[1],      0;
                Q2S = (2*(qS-Q2-delta2*Q2Dot))/(delta2*delta2);
                Q2I = (2*(qI-Q2-delta2*Q2Dot))/(delta2*delta2);

                // Add friction
                D = Fricc*QDot;
                h = h+D;

                D = Fricc*Q2Dot;
                h2 = h2+D;
                // Measure the errors
                e = pos - posD;
                e2 = pos2 - posD2;

                // task error w.r.t. orientation
                eQ = od4[0]*o3 - o4[0]*od3 + od3hat*o3;
                err << e , eQ;
                bA = - Kp*err - Kd*Ja*QDot - dJa*QDot;

                eQ2 = od4_2[0]*o3_2 - o4_2[0]*od3_2 + od3hat2*o3_2;
                err2 << e2 , eQ2;
                bA2 = - Kp*err2 - Kd*J2a*Q2Dot - dJ2a*Q2Dot;


                G = TrG.asDiagonal();
                /*
                lbA << -h,-h2, bA.block(3,0,3,1),bA2.block(3,0,3,1),  QI,Q2I;
                ubA << -h,-h2, bA.block(3,0,3,1),bA2.block(3,0,3,1),  QS,Q2S;

                A <<          H_Total       ,         -eye16          ,
                    Ja_Total.block(3,0,3,16),  zeros16.block(0,0,3,16),
                    Ja_Total.block(9,0,3,16),  zeros16.block(0,0,3,16),
                               eye16        ,         zeros16         ;
                */
                A <<          H_Total       ,         -eye16          ,
                                Ja_Total    ,  zeros16.block(0,0,12,16),
                               eye16        ,         zeros16         ;

                lbA << -h,-h2, bA,bA2,  QI,Q2I;
                ubA << -h,-h2, bA,bA2,  QS,Q2S;


                nWSR = 300;
                QProblem example(nV,cOns);
                example.setOptions( myOptions );
                example.init( G.data(),g.data(),A.data(),0,0,lbA.data(),ubA.data(), nWSR,0 , xOpt1.data() );
                // Get and print solution
                example.getPrimalSolution( xOpt1.data() );

                /*
                Ax = A*xOpt1;
                G2 = TrG2.asDiagonal();
                for  ( int j=0; j<QS.size(); j++ ){
                    UpBnd = false;
                    lwBnd = false;

                    if ( Ax(j+11) >= QS(j) )   UpBnd = true;
                    if ( Ax(j+11) <= QI(j) )   lwBnd = true;
                    if ( UpBnd == true)     QI(j) = QS(j);
                    if ( lwBnd == true)     QS(j) = QI(j);

                }
                ubA2 << Ax.head(11) ,QS, bA.block(0,0,3,1) ;
                lbA2 << Ax.head(11) ,QI, bA.block(0,0,3,1) ;

                A2   <<                 A                  ,         zeros19_3   ,
                           Ja.block(0,0,3,8),zeros68.block(0,0,3,8),           -eye3     ;



                nWSR2 = 300;
                QProblem example2{nV2,cOns2};
                example2.setOptions( myOptions2 );
                example2.init( G2.data(),g2.data(),A2.data(),0,0,lbA2.data(),ubA2.data(), nWSR2, 0 , xOpt2.data() );
                // Get and print solution
                example2.getPrimalSolution( xOpt2.data() );

                xOpt1 = xOpt2.head(nV);*/
                QDDot = xOpt1.head(Dof);
                QDot = QDot + QDDot*delta;
                Q = Q + QDot*delta;

                Q2DDot = xOpt1.segment(8,Dof);;
                Q2Dot = Q2Dot + Q2DDot*delta;
                Q2 = Q2 + Q2Dot*delta;







                // Set the initial configuration to Vrep
                //simxSetJointPosition(clientID, worldx, Q(0), simx_opmode_streaming);
                //simxSetJointPosition(clientID, worldy, Q(1), simx_opmode_streaming);
                //simxSetJointPosition(clientID, baseangle,Q(2), simx_opmode_streaming);
                simxSetJointPosition(clientID, joint1, Q(3), simx_opmode_streaming);
                simxSetJointPosition(clientID, joint2, Q(4), simx_opmode_streaming);
                simxSetJointPosition(clientID, joint3, Q(5), simx_opmode_streaming);
                simxSetJointPosition(clientID, joint4, Q(6), simx_opmode_streaming);
                simxSetJointPosition(clientID, joint5, Q(7), simx_opmode_streaming);
                //simxSetJointPosition(clientID, worldx2, Q2(0), simx_opmode_streaming);
                //simxSetJointPosition(clientID, worldy2, Q2(1), simx_opmode_streaming);
                //simxSetJointPosition(clientID, baseangle2,Q2(2), simx_opmode_streaming);
                simxSetJointPosition(clientID, joint1_2, Q2(3), simx_opmode_streaming);
                simxSetJointPosition(clientID, joint2_2, Q2(4), simx_opmode_streaming);
                simxSetJointPosition(clientID, joint3_2, Q2(5), simx_opmode_streaming);
                simxSetJointPosition(clientID, joint4_2, Q2(6), simx_opmode_streaming);
                simxSetJointPosition(clientID, joint5_2, Q2(7), simx_opmode_streaming);


                std::cout <<"\n"<<" Time [sec] "<< "\n"<< (i / (double) 100) << std::endl;
                std::cout << std::endl << "Errores" << std::endl;
                std::cout << "+-----------------------------------------" << std::endl
                    << "|  Error" << "\t" << "Valor"<< "\t\t\t" << "Actual"<< "\t\t\t" << "Deseada" << std::endl
                    << "|  eQw" << "\t\t" << "------" << "\t\t" << o4(0) <<"\t\t" << od4(0)<< std::endl
                    << "|  eQx" << "\t\t" << eQ(0) << "\t\t" << o4(1) <<"\t\t" << od4(1)<< std::endl
                    << "|  eQy" << "\t\t" << eQ(1) << "\t\t" << o4(2) <<"\t\t" << od4(2)<< std::endl
                    << "|  eQz" << "\t\t" << eQ(2) << "\t\t" << o4(3) <<"\t\t" << od4(3)<< std::endl
                    << "+-----------------------------------------" << std::endl;

                SavetoFile(dataFile, dataFile2,dataFile3,dataFile4,dataFile5,QDDot,QDot,Q,err,FT);
                msleep(3);
            }

            fclose(dataFile); fclose(dataFile2); fclose(dataFile3);fclose(dataFile4);fclose(dataFile5);
            //std::cout <<"\n"<<" FinAl Xopt1 = "<< "\n"<< xOpt1 << std::endl;

        }



    ~KUKA (){
        delete model;
        }
};

int main () {

    rbdl_check_api_version (RBDL_API_VERSION);
    //Class Robot KUKA Object Youbot
    KUKA Youbot;
    //KUKA Youbot2;
    int portNb = 19997;
    int clientID = simxStart("127.0.0.1", portNb, true, true, 5000, 5);

    Youbot.QPsolver(clientID);
    //Youbot2.QPsolver2(clientID);
    std::cout << "Done" << std::endl;
    return 0;
}


