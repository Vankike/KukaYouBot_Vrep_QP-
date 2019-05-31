/*
//! position and orientation feedback from Vrep successful -- dynamic environment -- E.E. rectangular design
//! Wheels velocity command successful
//! quaternion feedback - positive of escalar part -- 180 degrees task fail. -- Yuan Paper.

//! versiones de 2.0 en adelante conforman un control de fuerza para un solo robot kuka contra una pared en simulacion con vrep.
//! Funciona con la escena force_control_experiment.tt

//! version 2.1 aun contiene errores en el control de fuerza puesto que estan impuestas 3 tareas de posicion
//! en x,y,z del e.e. + la tarea de fuerza en el eje z local del sensor, se agrego la funcion euler2rotation, se agrego la orientacion
//! del sensor de fuerza.

//! version 2.2
//!Se agregó la librería Chrono para medir el time step y general time del programa.
//! a partir de esta version se crean actualizaciones de los headers, ahora se usan vrepInit2_2.cpp y vrepInit2_2.h
//! RECORDAR CAMBIAR ESTA VERSION EN EL .PRO CADA QUE SE UTILICE UNA VERSION DIFERENTE.
//! se guarda esta versión para hacer modificaciones con el delta_t, delay, etc.
//!
*/

#include <qpOASES.hpp>
#include <iostream>
#include <rbdl/rbdl.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <stdio.h>
#include<stdlib.h>
#include <time.h>
#include <chrono>
#include <vrepInit2_2.h>

extern "C" {
#include "extApi.h"
}
#include <rtf_common.h>
#include <rtfilter.h>
#include<gnuplot-iostream.h>

using namespace RigidBodyDynamics;
using namespace Eigen;
USING_NAMESPACE_QPOASES


Matrix3d Euler2Rotation(double x, double y, double z){
    Matrix3d R,X,Y,Z;

    R.setZero();
    X.setIdentity();
    Y.setIdentity();
    Z.setIdentity();
    X(1,1) = cos(x);
    X(1,2) = -sin(x);
    X(2,1) = sin(x);
    X(2,2) = cos(x);
    Y(0,0) = cos(y);
    Y(0,2) = sin(y);
    Y(2,0) = -sin(y);
    Y(2,2) = cos(y);

    Z(0,0) = cos(z);
    Z(0,1) = -sin(z);
    Z(1,0) = sin(z);
    Z(1,1) = cos(z);

    R = Z*Y*X;
    return R;
}

int SavetoFile(FILE *dataFile,FILE *dataFile2,FILE *dataFile3,FILE *dataFile4,FILE *dataFile5,
               VectorXd QDDot,VectorXd QDot, VectorXd Q, VectorXd err,VectorXd Datos){
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
    fprintf(dataFile5,"%.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f\t %.6f",Datos(0),Datos(1),Datos(2),Datos(3), Datos(4),Datos(5),Datos(6),Datos(7) );
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
    #define DofTot  8
    #define nV      16
    #define cOns	22
    #define nV2     22
    #define cOns2	22
    bool update_kinematics, UpBnd, lwBnd ,wheelsON;
    double l1,l2,l3,l4,l5,l6,l7, t0, tEnd, delta,delta2, KP, KD,Kwh,Krot, Num_Iter, delay,mObj,normfc,TotalProgramTime,step,
           a1,a2,b1,b2,c1,c2, PcPr_Norma, gainKI, ProdEscalar, Vmax, Vmin, EscalarVN, Vclamp,tt,vueltas,teOK;
    int Finaltime, NN, RegSteps, RegSteps2, nWSR, nWSR2;
    unsigned int body_a_id, body_b_id, body_c_id, body_d_id, body_e_id,  body_f_id, body_g_id, body_h_id,body_Obj_id;
    Body body_a, body_b, body_c, body_d, body_e, body_f, body_g, body_h, body_Obj;
    Joint joint_a, joint_b, joint_c, joint_d, joint_e, joint_f, joint_g, joint_h,joint_Obj;

    Vector3d r1, r2, r3, r4, r5, r6, r7, r8, CoM1, CoM2, CoM3, CoM4, CoM5, CoM6, CoM7, CoM8, rObj,bAc,
             JointAxisZ, dP, dPh, Z, w, dQ_13, e, eQ , o3, od3, pos, posD,CoMObj,OFs,
             e2, eQ2 , o3_2, od3_2, pos2, posD2, Normal,Pc,Pk,Pj,PjPc, PkPc, Pr, PcPr, Normal_Unitario,
             ForceSensed,TorqueSensed, ForceQP, VelocityRef, VCLAMP, VCLAMP_ANT,QCLAMP,ACLAMP, V_filter, xdt;
    Vector4d Quat, dQuat, o4, od4;


    VectorXd mass{Dof}, h{Dof}, qS{Dof},qI{Dof}, qM{Dof}, QS{Dof},QI{Dof},fr{Dof},D{Dof},Q{Dof}, QDot{Dof}, QDDot{Dof}, Vnu{6}, deriva{6},
             Datos{8},TrG{nV},g{nV},ub{nV},lb{nV},lbA{cOns},ubA{cOns},lbA2{cOns2},ubA2{cOns2},xOpt1{nV}, POSE{7},err{6}, bA{6},tkd{6},tkp{6},QObj{6},
             QDotObj{6},QDDotObj{6},comparador{2}, vdt{6}, adt{6};

    Math::Matrix3d Ti1, Ti2, Ti3,Ti4,Ti5, Ti6, Ti7,Ti8,E1, E2, E3, E4, E5, E6, E7, E8,TiObj,EObj;
    Matrix3d O, od3hat, eye3, zero3, dJthInv,
             O2, od3hat2,KI,RotFS;
    MatrixXd H{Dof,Dof}, Ja{6,Dof}, dJa{6,Dof}, Ag{6,6}, Jg{6, Dof}, dJg{6, Dof}, dJxInv{6, 6}, JxInv{6, 6}, Jeps{3,4}, G{nV,nV},
             Jth{3,4}, JthInv{4,3}, Fricc{Dof,Dof}, eye8{8,8},zeros8{8,8}, zeros3_8{3,8},zeros8_3{8,3},
             kd{6,6},kp{6,6}, Jc{3,Dof},dJc{3,Dof};


    Eigen::Matrix< double,cOns ,nV ,RowMajor> A;
    Eigen::Matrix< double,cOns2,nV2,RowMajor> A2;
    real_t eps,eps2;
    Model* model;
    Model* modelObj;
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
    struct dynaObj{
            VectorXd h{6},Pose{7};
            MatrixXd H{6,6},Ja{6,6},dJa{6,6};
        };
    qpOASES::Options myOptions;

  public:

    KUKA () { //! Constructor. Parameters of the KUKAyouBot

        /*******************************************************************************
        * Initial Values
        *******************************************************************************/
        wheelsON = true;

        Q  <<  0,  -1,  0 ,  0 , 0.2, -0.1, 0.02,  0;


        fr  << 6,  6 , 6, 0.1, 0.3,  0.3, 0.2,  0.07;

        tkp << 0.6, 0.6, 0.6, 1.2, 1.2, 1.2;
        tkd << 6, 6, 6, 6, 6, 6;

        tkp <<  1.4,1.4,1.4,1.4,1.4,1.4;
        tkd <<  12 , 12, 12, 12, 12, 12;
        //tkp << 40,,5,5,5,5;
       // tkd <<  20,20,20,20,20,20;


        Kwh = 8;
        Krot= 8;


        KI << 0.001,   0  ,     0,
                0  , 0.001,     0,
                0  ,   0  , 0.002;

        gainKI =  5.0000e-04;
        KP = 0.1;
        KD = 0.26;
        delta = 0.001;
        delta2 = 0.01;
        t0 = 0;
        tEnd = 80;
        delay = 0; // milisec.
        od4  << 0.7011 , 0, 0.7011, 0;
        Vmax = 0.05;
        Vmin = -0.05;
        tt = 1;
        vueltas = 1;
        teOK = 25000;

        Finaltime = tEnd;
        Num_Iter = (tEnd - t0)/delta;
        NN = Num_Iter;
        QDot.setZero();
        QDDot.setZero();

        QObj.setZero();
        QDotObj.setZero();
        h.setZero();
        Datos.setZero();
        ForceQP.setZero();
        H.setZero();

        kd.setZero();
        kp.setZero();
        eye8.setIdentity();

        zeros8.setZero();
        ACLAMP.setZero();
        QCLAMP.setZero();
        VCLAMP.setZero();
        V_filter.setZero();
        xdt.setZero();
        vdt.setZero();
        adt.setZero();

        Jg.setZero();
        dJg.setZero();
        zero3.setZero();
        Ag.setZero();
        eye3.setIdentity();

        xOpt1.setZero();

        Fricc  = fr.asDiagonal();
        kp = tkp.asDiagonal();
        kd = tkd.asDiagonal();
        for (int i = 7; i >= 0; i--)  J[i].G.setZero();
        update_kinematics = true;

        /*******************************************************************************
        * Joint Limits
        *******************************************************************************/
        qS <<  5,  5,  2*M_PI,  2.932,  1.553,  2.53,  1.78,  2.89;
        qI << -5, -5, -2*M_PI, -2.932, -1.1170, -2.62, -1.78, -2.89;
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
        //TrG(nV-1) = 0.9;
        TrG = TrG*0.9;
        g.setZero();
        G.setZero();

        /*******************************************************************************
        * Low pass butterworth filter
        *******************************************************************************/
        #define NCH		1
        #define FS		500	/* in Hz */
        #define CUTOFF		20	/* in Hz */
        #define CHUNKNS		1  /* number of samples samples*/
        #define RAMPDUR		0.1	/* in seconds */
        #define DURATION	10	/* in seconds */
        #define NPOLES	4

        /*******************************************************************************************************************
        * Model Parameters (Gravity, Link_Lengths, Action_Axis_Z, Masses, Centers_Of_Mass, X_T, Inertia_Tensioners, Bodies )
        ********************************************************************************************************************/
        modelObj = new Model();
        modelObj->gravity = Math::Vector3d (0.,0.,-9.81);
        CoMObj << 0.0, 0.0, 0.0; // T = 0.0034 (x3) version tensor inertia libro Murray
        mObj = 3.2;
        TiObj <<   0.0085, 0, 0,0,   0.0085, 0, 0, 0,   0.0085;
        EObj <<  1 , 0 , 0, 0 , 1 , 0,0 , 0 , 1;
        rObj <<  0, 0, 0;
        body_Obj = Body (mObj, CoMObj, TiObj );    joint_Obj = Joint(JointTypeFloatingBase);
        body_Obj_id = modelObj->AddBody(    0    , Math::SpatialTransform( EObj, rObj),  joint_Obj, body_Obj);

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

    dynamic HandC(VectorXd Q,VectorXd QDot){//!Return Inertia Matrix, Nonlinear terms, Analytic Jacobian and its derivative and the Pose

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

    Vector4d Quater(Matrix3d R){ //!std::cout<<"quaternion scalar"<<P.w()<<"Vector"<<std::endl<<P.x()<< std::endl;
            Quaterniond P(R);    Vector4d QuatEig;
            QuatEig(0) = P.w();     QuatEig(1) = P.x();     QuatEig(2) = P.y();     QuatEig(3) = P.z();
            return QuatEig;
        }

    Matrix3d skew(MatrixXd v) { //! Skewsymmetric Matrix Form
            Matrix3d S;
            S <<     0, -v(2),  v(1),
                  v(2),     0, -v(0),
                 -v(1),  v(0),     0;
            return S;
        }

    void QPsolver()
        {

            VectorXd resp(29);
            VectorXd Vref_x(NN);
            VectorXd Vref_y(NN);
            VectorXd Vref_z(NN);
            VectorXd ForceErrorX(NN);
            VectorXd ForceErrorY(NN);
            VectorXd ForceErrorZ(NN);

            hfilter filt = NULL;

            double input[CHUNKNS*NCH], output[CHUNKNS*NCH];
            double input2[CHUNKNS*NCH], output2[CHUNKNS*NCH];
            double input3[CHUNKNS*NCH], output3[CHUNKNS*NCH];
            // Create the butterworth filter
            normfc = (double)CUTOFF/(double)FS;
            filt = rtf_create_butterworth(NCH, RTF_DOUBLE, normfc, NPOLES, 0);
            if (filt == NULL){
                std::cout<<"No filt created"<<std::endl;
                exit(1);
            }

            // Open a .txt file to save data.
            FILE *dataFile = fopen("hqp_poses.txt","w");
            FILE *dataFile2= fopen("hqp_vel.txt","w");
            FILE *dataFile3= fopen("hqp_acc.txt","w");
            FILE *dataFile4= fopen("hqp_e.txt","w");
            FILE *dataFile5= fopen("hqp_Datos.txt","w");
            // structure dynamic
            dynamic ans;

            VREP::Simulador vrepCOM(Q,wheelsON,Kwh,Krot);
            resp = vrepCOM.connect(Q,QDot,wheelsON);

             Q(0) = resp(0);
             Q(1) = resp(1);
             Q(2) = resp(2);             
             pos<< resp(3), resp(4), resp(5);
             posD<< resp(6), resp(7), resp(8);
             ForceSensed << resp(9), resp(10), resp(11);
             TorqueSensed << resp(12), resp(13), resp(14);
             Pk << resp(15), resp(16), resp(17);
             Pj << resp(18), resp(19), resp(20);
             OFs << resp(21), resp(22), resp(23);
             Q(3)  = resp(24);
             Q(4)  = resp(25);
             Q(5)  = resp(26);
             Q(6)  = resp(27);
             Q(7)  = resp(28);
             RotFS = Euler2Rotation(OFs(0),OFs(1),OFs(2));
             ForceSensed = RotFS*(ForceSensed);

             Pc = posD;

             QCLAMP = posD;
             auto t_init = std::chrono::high_resolution_clock::now();

           for (int i=1;i<NN;i++){
                auto t_start = std::chrono::high_resolution_clock::now();
                ans = HandC(Q,QDot);

                H = ans.H;                 
                h = ans.h;

                Ja = ans.Ja;
                dJa = ans.dJa;
                Jc = Ja.block(0,0,3,8);
                dJc = dJa.block(0,0,3,8);

                POSE = ans.Pose;

                //pos = POSE.block(0,0,3,1);
                o4  = POSE.block(3,0,4,1); 
                o3 << o4[1],o4[2],o4[3];
                od3 << od4[1],od4[2],od4[3];
                od3hat << 0   , -od4[3],  od4[2],
                        od4[3],    0   , -od4[1],
                       -od4[2],  od4[1],     0;
                QS = (2*(qS-Q-delta2*QDot))/(delta2*delta2);
                QI = (2*(qI-Q-delta2*QDot))/(delta2*delta2);

                // Add friction
                D = Fricc*QDot;
                h = h+D;


                /*
                // Calculate the Normal Vector

                PkPc << Pc(0) - Pk(0), Pc(1) - Pk(1), Pc(2) - Pk(2);
                PjPc << Pc(0) - Pj(0), Pc(1) - Pj(1), Pc(2) - Pj(2);

                //! Asignamos un valor en el punto Pr menor  que el punto Pc en el eje X.
                Pr(0) = Pc(0)-1;

                a1 = PkPc(1);
                a2 = PjPc(1);
                b1 = PkPc(2);
                b2 = PjPc(2);
                c1 = PkPc(0)*Pc(0) + PkPc(1)*Pc(1) + PkPc(2)*Pc(2) - PkPc(0)*Pr(0);
                c2 = PjPc(0)*Pc(0) + PjPc(1)*Pc(1) + PjPc(2)*Pc(2) - PjPc(0)*Pr(0);

                Pr(1)=((c1*b2)-(c2*b1))/((a1*b2)-(a2*b1));
                Pr(2)=((a1*c2)-(a2*c1))/((a1*b2)-(a2*b1));

                PcPr << Pr(0) - Pc(0), Pr(1) - Pc(1), Pr(2) - Pc(2);
                PcPr_Norma = PcPr.norm();

                Normal_Unitario <<  PcPr(0)/PcPr_Norma, PcPr(1)/PcPr_Norma, PcPr(2)/PcPr_Norma;

                Normal = Normal_Unitario*-ForceSensed(2);

                ForceError = ForceQP - ForceSensed;

                ProdEscalar = ForceError(0)*Normal(0) + ForceError(1)*Normal(1) + ForceError(2)*Normal(2);

                VelocityRef =  Normal*ProdEscalar*gainKI;

                EscalarVN = VelocityRef(0)*Normal(0) + VelocityRef(1)*Normal(1) + VelocityRef(2)*Normal(2);

                comparador << Vmin , EscalarVN;
                Vclamp = comparador.maxCoeff();
                comparador << Vclamp , Vmax;
                Vclamp = comparador.minCoeff();
                */

                // #####################   FORCE CONTROL PART  ############################# //

                /*
                ForceErrorX(i-1) =  (-ForceSensed(0)) - (60);
                ForceErrorY(i-1) =    0 - ForceSensed(1);
                ForceErrorZ(i-1) =    0 - ForceSensed(2);
                
                //VelocityRef = KI*ForceError(i-1);
                VelocityRef(0) = ForceErrorX(i-1);
                VelocityRef(1) = 0.001*ForceErrorY(i-1);
                VelocityRef(2) = 0.001*ForceErrorZ(i-1);
                
                comparador << Vmin ,VelocityRef(0);
                VCLAMP(0) =  comparador.maxCoeff();
                comparador << Vmin ,VelocityRef(1);
                VCLAMP(1) =  comparador.maxCoeff();
                comparador << Vmin ,VelocityRef(2);
                VCLAMP(2) =  comparador.maxCoeff();

                comparador <<  Vmax  ,  VCLAMP(0);
                VCLAMP(0) =  comparador.minCoeff();
                comparador <<  Vmax  ,  VCLAMP(1);
                VCLAMP(1) =  comparador.minCoeff();
                comparador <<  Vmax  ,  VCLAMP(2);
                VCLAMP(2) =  comparador.minCoeff();

                Vref_z(i-1) = VCLAMP(2);

                input[0] = VCLAMP(0);
                input2[0] = VCLAMP(1);
                input3[0] = VCLAMP(2);

                // Filter the data
                rtf_filter(filt,input, output, CHUNKNS);
                // Filter the data
                rtf_filter(filt,input2, output2, CHUNKNS);
                // Filter the data
                rtf_filter(filt,input3, output3, CHUNKNS);

                V_filter(0) =  output[0];
                V_filter(1) =  output2[0];
                V_filter(2) =  output3[0];
                // V_filter = VCLAMP;



                if (i >= 2501){

                QCLAMP = QCLAMP + V_filter*delta;
                ACLAMP = (V_filter - VCLAMP_ANT)/delta ;
                }

                // velocity
                VCLAMP_ANT = V_filter;

                */

                // #####################   QPOASES PROBLEMS  ############################# //

                G = TrG.asDiagonal();

                if (i < teOK){

                    // Measure the errors
                    e = pos - posD;

                    // task error w.r.t. orientation
                    eQ = od4[0]*o3 - o4[0]*od3 + od3hat*o3;
                    err << e , eQ;
                    Vnu = Ja*QDot;
                    deriva = dJa*QDot;
                    bA = - kp*err - kd*Vnu - deriva;

                    std::cout <<"\n"<<" Time [sec] "<< "\n"<< (i / (double) 1000) << std::endl;
                  /*  std::cout << std::endl << "Errores" << std::endl;
                    std::cout << "+-----------------------------------------" << std::endl
                        << "|  Error" << "\t" << "Valor"<< "\t\t\t" << "Actual"<< "\t\t\t" << "Deseada" << std::endl
                        << "|  ex" << "\t\t" << e(0) << "\t\t" << pos(0) <<"\t\t" << posD(0)<< std::endl
                        << "|  ey" << "\t\t" << e(1) << "\t\t" << pos(1) <<"\t\t" << posD(1)<< std::endl
                        << "|  ez" << "\t\t" << e(2) << "\t\t" << pos(2) <<"\t\t" << posD(2)<< std::endl
                        << "+-----------------------------------------" << std::endl;*/
                 }

                if (i >= teOK){
                    /* this is not oK
                    xdt <<  cos(tt),
                            sin(tt),
                                 0.4610;

                    vdt << -tt*sin(tt),
                            tt*cos(tt),
                                 0,
                                 0,
                                 0,
                                 0;
                    adt <<  -tt*tt*cos(tt),
                            -tt*tt*sin(tt),
                                 0,
                                 0,
                                 0,
                                 0;
                        */
                    xdt <<  (double)log(tt)/(double)10,
                            (double)log(tt)/(double)10,
                                 0.4610;

                    vdt << (double)1/((double)10*tt),
                           (double)1/((double)10*tt),
                                 0,
                                 0,
                                 0,
                                 0;

                    adt <<  -(double)1/((double)10*pow(tt,2)),
                            -(double)1/((double)10*pow(tt,2)),
                                 0,
                                 0,
                                 0,
                                 0;
                    // Measure the errors
                    e = pos - xdt;

                    // task error w.r.t. orientation
                    eQ = od4[0]*o3 - o4[0]*od3 + od3hat*o3;
                    err << e , eQ;
                    Vnu = Ja*QDot - vdt ;
                    deriva = dJa*QDot;
                    bA = - kp*err - kd*Vnu - deriva + adt;

                    //tt = tt + ((vueltas*2*M_PI)/(Num_Iter-teOK));
                    //tt = ((2*M_PI)/(Num_Iter-teOK));
                     tt = tt + 1;

                    std::cout <<"\n"<<" Time [sec] "<< "\n"<< (i / (double) 1000) << std::endl;
                    std::cout << std::endl << "Errores 2.0 "  <<std::endl;
                    std::cout << "+-----------------------------------------" << std::endl
                        << "|  Error" << "\t" << "Valor"<< "\t\t\t" << "Actual"<< "\t\t\t" << "Deseada" << std::endl
                        << "|  ex" << "\t\t" << e(0) << "\t\t" << pos(0) <<"\t\t" << xdt(0)<< std::endl
                        << "|  ey" << "\t\t" << e(1) << "\t\t" << pos(1) <<"\t\t" << xdt(1)<< std::endl
                        << "|  ez" << "\t\t" << e(2) << "\t\t" << pos(2) <<"\t\t" << xdt(2)<< std::endl
                        << "+-----------------------------------------" << std::endl;
                }


                lbA << -h,bA, QI;
                ubA << -h,bA, QS;
                A <<         H         , -eye8    ,
                     Ja.block(0,0,3,8) , zeros3_8 ,
                     Ja.block(3,0,3,8) , zeros3_8 ,
                            eye8       , zeros8   ;

                nWSR = 300;
                QProblem example(nV,cOns);
                example.setOptions( myOptions );
                example.init( G.data(),g.data(),A.data(),0,0,lbA.data(),ubA.data(), nWSR,0 , xOpt1.data() );
                // Get and print solution
                example.getPrimalSolution( xOpt1.data() );

                 // #####################   NUMERICAL INTEGRATION PART & UPDATE VALUES  ############################# //

                QDDot = xOpt1.head(Dof);
                QDot = QDot + QDDot*delta;
                Q = Q + QDot*delta;

                // Set the configuration to Vrep
                resp = vrepCOM.connect(Q,QDot,wheelsON);
                Q(0) = resp(0);
                Q(1) = resp(1);
                Q(2) = resp(2);
                pos<< resp(3), resp(4), resp(5);
                posD<< resp(6), resp(7), resp(8);
                ForceSensed << resp(9), resp(10), resp(11);
                TorqueSensed << resp(12), resp(13), resp(14);
                Pk << resp(15), resp(16), resp(17);
                Pj << resp(18), resp(19), resp(20);
                OFs << resp(21), resp(22), resp(23);

                Q(3)  = resp(24);
                Q(4)  = resp(25);
                Q(5)  = resp(26);
                Q(6)  = resp(27);
                Q(7)  = resp(28);

                RotFS = Euler2Rotation(OFs(0),OFs(1),OFs(2));
                ForceSensed = RotFS*ForceSensed;

                Pc = posD;

                //std::cout << i <<"\n"<<"FORCE SENSOR = "<< "\n"<< ForceSensed << std::endl;
                Datos << ForceSensed , ForceSensed, VCLAMP(0) , V_filter(0);
                /*
                std::cout <<"\n"<<" Time [sec] "<< "\n"<< (i / (double) 100) << std::endl;
                std::cout << std::endl << "Errores" << std::endl;
                std::cout << "+-----------------------------------------" << std::endl
                    << "|  Error" << "\t" << "Valor"<< "\t\t\t" << "Actual"<< "\t\t\t" << "Deseada" << std::endl
                    << "|  eQw" << "\t\t" << "------" << "\t\t" << o4(0) <<"\t\t" << od4(0)<< std::endl
                    << "|  eQx" << "\t\t" << eQ(0) << "\t\t" << o4(1) <<"\t\t" << od4(1)<< std::endl
                    << "|  eQy" << "\t\t" << eQ(1) << "\t\t" << o4(2) <<"\t\t" << od4(2)<< std::endl
                    << "|  eQz" << "\t\t" << eQ(2) << "\t\t" << o4(3) <<"\t\t" << od4(3)<< std::endl
                    << "+-----------------------------------------" << std::endl;
                */


                SavetoFile(dataFile, dataFile2,dataFile3,dataFile4,dataFile5,QDDot,QDot,Q,err,Datos);
                msleep(delay);

                auto t_end = std::chrono::high_resolution_clock::now();
                step=std::chrono::duration<double>(t_end-t_start).count();
                std::cout<< "Step time = " << step<< " s\n"
                <<"Current time = "<<std::chrono::duration<double>(t_end-t_init).count()<<std::endl<<std::endl;
                TotalProgramTime=std::chrono::duration<double>(t_end-t_init).count();


            }
            fclose(dataFile); fclose(dataFile2); fclose(dataFile3);fclose(dataFile4);fclose(dataFile5);

            /*
            bool interactive = true;

            Gnuplot gp;
            //Gnuplot ForcePlotx;
            gp << "plot '-' notitle with lines \n";
            gp << Vref_z << "\ne\e" << std::endl;


            //ForcePlotx << "plot '-' notitle with lines \n";
            //ForcePlotx << ForceErrorX << "\ne\e" << std::endl;


            if(interactive){
                std::cout<<"Presione enter para terminar."<<std::endl;
                std::cin.get();
            }
            */
            //msleep(2000);

        }
    ~KUKA (){
        delete model;
        delete modelObj;
        }
};

int main () {

    rbdl_check_api_version (RBDL_API_VERSION);

    KUKA Youbot;
    Youbot.QPsolver();

    std::cout <<"\n"<< "Done  "   <<std::endl;

    return 0;
}
