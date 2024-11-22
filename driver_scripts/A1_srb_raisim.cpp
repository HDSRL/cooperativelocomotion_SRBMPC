//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "raisim/OgreVis.hpp"
#include "randyImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"
#include "helper2.hpp"

//HDSRL header
#include "LocoWrapper.hpp"
//#include "DeePC_MPC.hpp"
#include "a1_controller/locomotion_planner.h"
#include "shared_structs.hpp"
#include "OtherUtils.hpp"

void setupCallback() {
    raisim::OgreVis *vis = raisim::OgreVis::get();

    /// light 1
    //Ogre::Vector3 lightdir(-3,3,-0.5); // Light shines on ROBOTS top/front/left side
    //Ogre::Vector3 lightdir(-1,-1,-1);
    Ogre::Vector3 lightdir(-1,1,-1);
    lightdir.normalise();
    vis->getLight()->setDiffuseColour(1, 1, 1);
    vis->getLight()->setCastShadows(false); // this needs to be changed to true for enabling shadow in sim 
    // if you want to enable the shadow, disturbance extforcearrow vis object needs to be commented out
    vis->getLightNode()->setDirection({lightdir});

    vis->addResourceDirectory(raisim::loadResource("material"));
    vis->loadMaterialFile("myMaterials.material");
    vis->addResourceDirectory(vis->getResourceDir() + "/material/skybox/violentdays");
    vis->loadMaterialFile("violentdays.material");

    // load my textures
    vis->addResourceDirectory("/home/seop/git_repo_temp/cooperativelocomotion_SRBMPC/material_collection");
    vis->loadMaterialFile("collection.material");

    /// shdow setting
    vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
    vis->getSceneManager()->setShadowTextureSettings(2048, 3);

    /// scale related settings!! Please adapt it depending on your map size
    // beyond this distance, shadow disappears
    vis->getSceneManager()->setShadowFarDistance(10);
    // size of contact points and contact forces
    vis->setContactVisObjectSize(0.03, 0.6);
    // speed of camera motion in freelook mode
    vis->setCameraSpeed(300);
    vis->getCameraMan()->setTopSpeed(5);

    Ogre::Quaternion quat;
    quat.FromAngleAxis(Ogre::Radian(M_PI_2), {1., 0, 0});
    vis->getSceneManager()->setSkyBox(true,"black",500,true,quat,
                                    Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);
}

void disturbance(std::vector<raisim::ArticulatedSystem *> A1,
                 size_t agnum,
                 std::map<std::string, raisim::VisualObject> &objList,
                 const std::string& objname,
                 size_t start,size_t stop,size_t ctrlTick){
    Eigen::VectorXd pos = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1);
    Eigen::VectorXd vel = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    A1[agnum]->getState(pos, vel);
    
    // ====================== External Forces ====================== //
    raisim::Vec<3> extForce;
    raisim::Mat<3,3> rot;
    raisim::Vec<3> dir;
    if (ctrlTick>=start && ctrlTick<stop){
        extForce = {0,40,0}; // Pulse
        // extForce = {50*sin(4*ctrlTick*simfreq_raisim),0,0}; // Fwd Sine
        //extForce = {0,20*sin(4*ctrlTick*simfreq_raisim),0}; // Lat Sine
        extForce = {20*sin((6+2*agnum)*ctrlTick*simfreq_raisim), 20*sin((4+5*agnum)*ctrlTick*simfreq_raisim), 20*sin((4+3*agnum)*ctrlTick*simfreq_raisim)}; // Lat Sine
    } else{
        extForce = {0,0,0};
    }
    A1[agnum]->setExternalForce(0,extForce);
    dir = extForce;
    dir /= dir.norm();
    raisim::zaxisToRotMat(dir, rot);
    objList[objname].offset = {pos(0),pos(1),pos(2)};
    objList[objname].scale = {0.2,0.2,0.015*extForce.norm()};
    objList[objname].rotationOffset = rot;
}

void controller(std::vector<raisim::ArticulatedSystem *> A1, 
                size_t totalagentnumber, size_t eachagentnumber,
                LocomotionPlanner* loco_pln,
                LocoWrapper *loco_obj,
                Eigen::MatrixXd agentinitial){
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// INITIALIZE
    /////////////////////////////////////////////////////////////////////
    static size_t controlTick_agent[4] = {0, 0, 0, 0};
    controlTick_agent[eachagentnumber]++;

    size_t loco_kind = TROT;                        // Gait pattern to use
    size_t pose_kind = POSE_ROLL;                   // Pose type to use (if loco_kind is set to POSE)
    size_t settling = 0.2*ctrlHz;                   // Settling down
    size_t duration = 0.8*ctrlHz;                   // Stand up
    size_t loco_start = settling + duration;        // Start the locomotion pattern

    loco_pln->timeSetup(duration, loco_start);

    double *tau;
    Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    Eigen::VectorXd jointPosTotal = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1);
    Eigen::VectorXd jointVelTotal = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    Eigen::VectorXd jointPosTotal_opposite = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1);
    Eigen::VectorXd jointVelTotal_opposite = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    static int conIndDes[4] = {1,1,1,1};
    static int conIndDes_opposite[4] = {1,1,1,1};

    // getting result and setup information from srb mpc
    Eigen::Matrix<double, 24, 1> optx = Eigen::Matrix<double, 24,1>::Zero(24,1);
    Eigen::Matrix<double, 24, 1> optu = Eigen::Matrix<double, 24,1>::Zero(24,1);
    Eigen::MatrixXd desiredstate = Eigen::MatrixXd::Zero(36,1);

    raisim::Mat<3,3> rotMat, rotMat_opposite;
    Eigen::Matrix3d rotE, rotE_opposite;
    double rotArr[9], rotArr_opposite[9];
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// UPDATE STATE
    /////////////////////////////////////////////////////////////////////
    // ================================ //
    // ========= subject agent ======== //
    // ================================ //
    A1[eachagentnumber]->getState(jointPosTotal, jointVelTotal);
    A1[eachagentnumber]->getBaseOrientation(rotMat);
    for(size_t i=0;i<9;i++){
        rotArr[i] = rotMat[i];
        rotE(i) = rotMat[i];
    }
    jointVelTotal.segment(3,3) = rotE.transpose()*jointVelTotal.segment(3,3); // convert to body frame, like robot measurements

    double jpos[18], jvel[18];
    Eigen::Matrix<double, 3, 1> eul;
    Eigen::Matrix<double, 4, 1> quat;
    quat = jointPosTotal.block(3,0,4,1);
    quat_to_XYZ(quat,eul);
    for(size_t i=0; i<3; ++i){
        jpos[i] = jointPosTotal(i);
        jvel[i] = jointVelTotal(i);
        jpos[i+3] = eul(i);
        jvel[i+3] = jointVelTotal(i+3);
    }
    for(size_t i=6; i<18; ++i){
        jpos[i] = jointPosTotal(i+1);
        jvel[i] = jointVelTotal(i);
    } 
    int force[4] = {0};
    for(auto &con: A1[eachagentnumber]->getContacts()){
        int conInd = con.getlocalBodyIndex();
        if (conInd%3!=0 || conInd>12) continue;
            force[conInd/3-1] = 21;
    }
    kinEst_multi(force,conIndDes,jpos,jvel,rotE,eachagentnumber,agentinitial);

    // ================================ //
    // ======== opposite agent ======== //
    // ================================ //
    double jpos_opposite[18], jvel_opposite[18];
    Eigen::Matrix<double, 3, 1> eul_opposite;
    Eigen::Matrix<double, 4, 1> quat_opposite;

    if(totalagentnumber == 2){
        A1[1-eachagentnumber]->getState(jointPosTotal_opposite, jointVelTotal_opposite);
        A1[1-eachagentnumber]->getBaseOrientation(rotMat_opposite);
        for(size_t i=0;i<9;i++){
            rotArr_opposite[i] = rotMat_opposite[i];
            rotE_opposite(i) = rotMat_opposite[i];
        }
        jointVelTotal_opposite.segment(3,3) = rotE_opposite.transpose()*jointVelTotal_opposite.segment(3,3); // convert to body frame, like robot measurements

        quat_opposite = jointPosTotal_opposite.block(3,0,4,1);
        quat_to_XYZ(quat_opposite,eul_opposite);
        for(size_t i=0; i<3; ++i){
            jpos_opposite[i] = jointPosTotal_opposite(i);
            jvel_opposite[i] = jointVelTotal_opposite(i);
            jpos_opposite[i+3] = eul_opposite(i);
            jvel_opposite[i+3] = jointVelTotal_opposite(i+3);
        }
        for(size_t i=6; i<18; ++i){
            jpos_opposite[i] = jointPosTotal_opposite(i+1);
            jvel_opposite[i] = jointVelTotal_opposite(i);
        } 
        int force_opposite[4] = {0};
        for(auto &con: A1[1-eachagentnumber]->getContacts()){
            int conInd_opposite = con.getlocalBodyIndex();
            if (conInd_opposite%3!=0 || conInd_opposite>12) continue;
                force_opposite[conInd_opposite/3-1] = 21;
        }
        //kinEst(force,conIndDes,jpos,jvel,rotE);
        kinEst_multi(force_opposite,conIndDes_opposite,jpos_opposite,jvel_opposite,rotE_opposite,1-eachagentnumber,agentinitial);
    }

    // ========================================== //
    // ======== agents' state collection ======== //
    // ========================================== //
    Eigen::Vector3d body0pos_e = Eigen::Vector3d::Zero();
    Eigen::Matrix<double,3,3> body0rot_e = Eigen::Matrix<double,3,3>::Zero(3,3);
    Eigen::Vector3d body0vel_e = Eigen::Vector3d::Zero();
    Eigen::Vector3d body0omega_e = Eigen::Vector3d::Zero();

    Eigen::Vector3d body1pos_e = Eigen::Vector3d::Zero();
    Eigen::Matrix<double,3,3> body1rot_e = Eigen::Matrix<double,3,3>::Zero(3,3);
    Eigen::Vector3d body1vel_e = Eigen::Vector3d::Zero();
    Eigen::Vector3d body1omega_e = Eigen::Vector3d::Zero();

    if(eachagentnumber == 0){
        for(size_t i = 0; i<3; i++){
            body0pos_e(i) = jpos[i];
            body0vel_e(i) = jvel[i];
            body0omega_e(i) = jvel[i+3];
            body1pos_e(i) = jpos_opposite[i];
            body1vel_e(i) = jvel_opposite[i];
            body1omega_e(i) = jvel_opposite[i+3];
        }
        body0rot_e = rotE;
        body1rot_e = rotE_opposite;
    }

    if(eachagentnumber == 1){
        for(size_t i = 0; i<3; i++){
            body1pos_e(i) = jpos[i];
            body1vel_e(i) = jvel[i];
            body1omega_e(i) = jvel[i+3];
            body0pos_e(i) = jpos_opposite[i];
            body0vel_e(i) = jvel_opposite[i];
            body0omega_e(i) = jvel_opposite[i+3];
        }
        body1rot_e = rotE;
        body0rot_e = rotE_opposite;
    }


    /////////////////////////////////////////////////////////////////////
    //////////////////////////// CONTROL
    /////////////////////////////////////////////////////////////////////
    // Update the desired torques from LL controller
    if(controlTick_agent[eachagentnumber] < settling){ // Settle down
        double temp[18] = {0};
        tau = temp;
        loco_obj->initStandVars(jointPosTotal.block(0,0,3,1),(int)duration,pose_kind);
    }

    else if(controlTick_agent[eachagentnumber] >= settling & controlTick_agent[eachagentnumber] <= loco_start){ // Start standing
        Eigen::Matrix<double, 12, 1> comDes;
        comDes.setZero();
        comDes.block(0,0,3,1) << agentinitial(eachagentnumber,0),agentinitial(eachagentnumber,1), 0.26;

        loco_obj->setDesired(comDes);
        loco_obj->calcTau(jpos,jvel,rotArr,force,STAND,controlTick_agent[eachagentnumber]);
        tau = loco_obj->getTorque();
    }

    else if(controlTick_agent[eachagentnumber] > loco_start){ // Start locomotion
        if(totalagentnumber == 2){
            loco_pln->getRobotState(body0pos_e, body1pos_e, body0vel_e, body1vel_e, body0rot_e, body1rot_e, body0omega_e, body1omega_e, controlTick_agent[eachagentnumber]);
            loco_pln->impactDetection_dist();
            optu.block(12*eachagentnumber,0,12,1) = loco_pln->forceFFdist();
            desiredstate = loco_pln->desiredstate();
        }

        if(totalagentnumber == 1){
            Eigen::Matrix<double, 12, 1> comDes;
            comDes.setZero();
            comDes.block(3,0,3,1) << 0.3, 0.0, 0;
            comDes.block(0,0,3,1) << jpos[0] + comDes(3)*0.001, jpos[1] + comDes(4)*0.001, 0.28;
            loco_obj->setDesired(comDes);
            loco_obj->calcTau(jpos,jvel,rotArr,force,loco_kind,controlTick_agent[eachagentnumber]);
        }
        else if(totalagentnumber == 2){
            Eigen::Matrix<double, 12, 1> comDes = Eigen::Matrix<double, 12, 1>::Zero(12,1);
            Eigen::Matrix<double, 12, 1> fDes = Eigen::Matrix<double, 12, 1>::Zero(12,1);
            if(eachagentnumber == 0){
                comDes.block(0,0,3,1) = desiredstate.block(0,0,3,1);
                comDes.block(3,0,3,1) = loco_pln->desiredcomvel(eachagentnumber);
                
                // == use this force command always == //
                fDes = optu.block(0,0,12,1);
            }
            else if(eachagentnumber == 1){
                comDes.block(0,0,3,1) = desiredstate.block(3,0,3,1);
                comDes.block(3,0,3,1)  = loco_pln->desiredcomvel(eachagentnumber);

                fDes = optu.block(12,0,12,1);
            }
            loco_obj->setDesired(comDes, fDes);
            loco_obj->calcTau(jpos,jvel,rotArr,force,loco_kind,controlTick_agent[eachagentnumber]);
        }

        tau = loco_obj->getTorque();

    }

    
    const int* contactMat = loco_obj->getConDes();
    memcpy(conIndDes,                 contactMat, 4*sizeof(int));

    jointTorqueFF = Eigen::Map< Eigen::Matrix<double,18,1> >(tau,18);
    jointTorqueFF.block(0,0,6,1).setZero();

    // Set the desired torques
    A1[eachagentnumber]->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    A1[eachagentnumber]->setGeneralizedForce(jointTorqueFF);
};

void controller2(std::vector<raisim::ArticulatedSystem *> A1, 
                size_t totalagentnumber, size_t eachagentnumber,
                LocomotionPlanner* loco_pln,
                LocoWrapper *loco_obj,
                LocoWrapper *loco_obj_opposite,
                Eigen::MatrixXd agentinitial){
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// INITIALIZE
    /////////////////////////////////////////////////////////////////////
    static size_t controlTick_agent[4] = {0, 0, 0, 0};
    controlTick_agent[eachagentnumber]++;

    size_t loco_kind = TROT;                        // Gait pattern to use
    size_t pose_kind = POSE_ROLL;                   // Pose type to use (if loco_kind is set to POSE)
    size_t settling = 0.2*ctrlHz;                   // Settling down
    size_t duration = 0.8*ctrlHz;                   // Stand up
    size_t loco_start = settling + duration;        // Start the locomotion pattern

    loco_pln->timeSetup(duration, loco_start);

    double *tau;
    Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    Eigen::VectorXd jointPosTotal = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1);
    Eigen::VectorXd jointVelTotal = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    Eigen::VectorXd jointPosTotal_opposite = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1);
    Eigen::VectorXd jointVelTotal_opposite = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    static int conIndDes[4] = {1,1,1,1};
    static int conIndDes_opposite[4] = {1,1,1,1};

    // getting result and setup information from srb mpc
    Eigen::Matrix<double, 24, 1> optx = Eigen::Matrix<double, 24,1>::Zero(24,1);
    Eigen::Matrix<double, 24, 1> optu = Eigen::Matrix<double, 24,1>::Zero(24,1);
    Eigen::MatrixXd desiredstate = Eigen::MatrixXd::Zero(36,1);

    raisim::Mat<3,3> rotMat, rotMat_opposite;
    Eigen::Matrix3d rotE, rotE_opposite;
    double rotArr[9], rotArr_opposite[9];
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// UPDATE STATE
    /////////////////////////////////////////////////////////////////////
    // ================================ //
    // ========= subject agent ======== //
    // ================================ //
    A1[eachagentnumber]->getState(jointPosTotal, jointVelTotal);
    A1[eachagentnumber]->getBaseOrientation(rotMat);
    for(size_t i=0;i<9;i++){
        rotArr[i] = rotMat[i];
        rotE(i) = rotMat[i];
    }
    jointVelTotal.segment(3,3) = rotE.transpose()*jointVelTotal.segment(3,3); // convert to body frame, like robot measurements

    double jpos[18], jvel[18];
    Eigen::Matrix<double, 3, 1> eul;
    Eigen::Matrix<double, 4, 1> quat;
    quat = jointPosTotal.block(3,0,4,1);
    quat_to_XYZ(quat,eul);
    for(size_t i=0; i<3; ++i){
        jpos[i] = jointPosTotal(i);
        jvel[i] = jointVelTotal(i);
        jpos[i+3] = eul(i);
        jvel[i+3] = jointVelTotal(i+3);
    }
    for(size_t i=6; i<18; ++i){
        jpos[i] = jointPosTotal(i+1);
        jvel[i] = jointVelTotal(i);
    } 
    int force[4] = {0};
    for(auto &con: A1[eachagentnumber]->getContacts()){
        int conInd = con.getlocalBodyIndex();
        if (conInd%3!=0 || conInd>12) continue;
            force[conInd/3-1] = 21;
    }
    //kinEst(force,conIndDes,jpos,jvel,rotE);
    //kinEst_multi(force,conIndDes,jpos,jvel,rotE,eachagentnumber,agentinitial);

    // ================================ //
    // ======== opposite agent ======== //
    // ================================ //
    double jpos_opposite[18], jvel_opposite[18];
    Eigen::Matrix<double, 3, 1> eul_opposite;
    Eigen::Matrix<double, 4, 1> quat_opposite;

    if(totalagentnumber == 2){
        A1[1-eachagentnumber]->getState(jointPosTotal_opposite, jointVelTotal_opposite);
        A1[1-eachagentnumber]->getBaseOrientation(rotMat_opposite);
        for(size_t i=0;i<9;i++){
            rotArr_opposite[i] = rotMat_opposite[i];
            rotE_opposite(i) = rotMat_opposite[i];
        }
        jointVelTotal_opposite.segment(3,3) = rotE_opposite.transpose()*jointVelTotal_opposite.segment(3,3); // convert to body frame, like robot measurements

        quat_opposite = jointPosTotal_opposite.block(3,0,4,1);
        quat_to_XYZ(quat_opposite,eul_opposite);
        for(size_t i=0; i<3; ++i){
            jpos_opposite[i] = jointPosTotal_opposite(i);
            jvel_opposite[i] = jointVelTotal_opposite(i);
            jpos_opposite[i+3] = eul_opposite(i);
            jvel_opposite[i+3] = jointVelTotal_opposite(i+3);
        }
        for(size_t i=6; i<18; ++i){
            jpos_opposite[i] = jointPosTotal_opposite(i+1);
            jvel_opposite[i] = jointVelTotal_opposite(i);
        } 
        int force_opposite[4] = {0};
        for(auto &con: A1[1-eachagentnumber]->getContacts()){
            int conInd_opposite = con.getlocalBodyIndex();
            if (conInd_opposite%3!=0 || conInd_opposite>12) continue;
                force_opposite[conInd_opposite/3-1] = 21;
        }
        //kinEst(force,conIndDes,jpos,jvel,rotE);
        //kinEst_multi(force_opposite,conIndDes_opposite,jpos_opposite,jvel_opposite,rotE_opposite,1-eachagentnumber,agentinitial);
    }

    // ========================================== //
    // ======== agents' state collection ======== //
    // ========================================== //
    Eigen::Vector3d body0pos_e = Eigen::Vector3d::Zero();
    Eigen::Matrix<double,3,3> body0rot_e = Eigen::Matrix<double,3,3>::Zero(3,3);
    Eigen::Vector3d body0vel_e = Eigen::Vector3d::Zero();
    Eigen::Vector3d body0omega_e = Eigen::Vector3d::Zero();

    Eigen::Vector3d body1pos_e = Eigen::Vector3d::Zero();
    Eigen::Matrix<double,3,3> body1rot_e = Eigen::Matrix<double,3,3>::Zero(3,3);
    Eigen::Vector3d body1vel_e = Eigen::Vector3d::Zero();
    Eigen::Vector3d body1omega_e = Eigen::Vector3d::Zero();

    if(eachagentnumber == 0){
        for(size_t i = 0; i<3; i++){
            body0pos_e(i) = jpos[i];
            body0vel_e(i) = jvel[i];
            body0omega_e(i) = jvel[i+3];
            body1pos_e(i) = jpos_opposite[i];
            body1vel_e(i) = jvel_opposite[i];
            body1omega_e(i) = jvel_opposite[i+3];
        }
        body0rot_e = rotE;
        body1rot_e = rotE_opposite;
    }

    if(eachagentnumber == 1){
        for(size_t i = 0; i<3; i++){
            body1pos_e(i) = jpos[i];
            body1vel_e(i) = jvel[i];
            body1omega_e(i) = jvel[i+3];
            body0pos_e(i) = jpos_opposite[i];
            body0vel_e(i) = jvel_opposite[i];
            body0omega_e(i) = jvel_opposite[i+3];
        }
        body1rot_e = rotE;
        body0rot_e = rotE_opposite;
    }


    /////////////////////////////////////////////////////////////////////
    //////////////////////////// CONTROL
    /////////////////////////////////////////////////////////////////////

    // Update the desired torques from LL controller
    if(controlTick_agent[eachagentnumber] < settling){ // Settle down
        double temp[18] = {0};
        tau = temp;
        loco_obj->initStandVars(jointPosTotal.block(0,0,3,1),(int)duration,pose_kind);
    }

    else if(controlTick_agent[eachagentnumber] >= settling & controlTick_agent[eachagentnumber] <= loco_start){ // Start standing
        Eigen::Matrix<double, 12, 1> comDes;
        comDes.setZero();
        comDes.block(0,0,3,1) << agentinitial(eachagentnumber,0),agentinitial(eachagentnumber,1), 0.26;
        loco_obj->setDesired(comDes);
        loco_obj->calcTau(jpos,jvel,rotArr,force,STAND,controlTick_agent[eachagentnumber]);
        tau = loco_obj->getTorque();
    }

    else if(controlTick_agent[eachagentnumber] > loco_start){ // Start locomotion

        if(totalagentnumber == 2){
            //loco_pln->getRobotState(body0pos_e, body1pos_e, body0vel_e, body1vel_e, body0rot_e, body1rot_e, body0omega_e, body1omega_e, controlTick_agent[eachagentnumber]);
            if(eachagentnumber == 0){
                loco_pln->getRobotStatewindx(body0pos_e, body1pos_e, body0vel_e, body1vel_e, 
                                            body0rot_e, body1rot_e, body0omega_e, body1omega_e, 
                                            controlTick_agent[eachagentnumber], conIndDes, conIndDes_opposite);
            }else if(eachagentnumber == 1){
                loco_pln->getRobotStatewindx(body0pos_e, body1pos_e, body0vel_e, body1vel_e, 
                                            body0rot_e, body1rot_e, body0omega_e, body1omega_e, 
                                            controlTick_agent[eachagentnumber], conIndDes_opposite, conIndDes);
            }

            loco_pln->getRobotState(body0pos_e, body1pos_e, body0vel_e, body1vel_e, body0rot_e, body1rot_e, body0omega_e, body1omega_e, controlTick_agent[eachagentnumber]);
            loco_pln->impactDetection_dist();
            optu.block(12*eachagentnumber,0,12,1) = loco_pln->forceFFdist();
            desiredstate = loco_pln->desiredstate();
        }

        if(totalagentnumber == 1){
            Eigen::Matrix<double, 12, 1> comDes;
            comDes.setZero();
            comDes.block(3,0,3,1) << 0.3, 0.0, 0;
            //comDes.block(0,0,3,1) << agentinitial(eachagentnumber,0),agentinitial(eachagentnumber,1), 0.26;
            comDes.block(0,0,3,1) << jpos[0] + comDes(3)*0.001, jpos[1] + comDes(4)*0.001, 0.28;
            loco_obj->setDesired(comDes);
            loco_obj->calcTau(jpos,jvel,rotArr,force,loco_kind,controlTick_agent[eachagentnumber]);
        }
        else if(totalagentnumber == 2){
            Eigen::Matrix<double, 12, 1> comDes = Eigen::Matrix<double, 12, 1>::Zero(12,1);
            Eigen::Matrix<double, 12, 1> fDes = Eigen::Matrix<double, 12, 1>::Zero(12,1);
            if(eachagentnumber == 0){
                comDes.block(0,0,3,1) = desiredstate.block(0,0,3,1); //optx.block(0,0,3,1);// combination?
                comDes.block(3,0,3,1) = loco_pln->desiredcomvel(eachagentnumber);
                
                fDes = optu.block(0,0,12,1);
            }
            else if(eachagentnumber == 1){
                comDes.block(0,0,3,1) = desiredstate.block(3,0,3,1); //optx.block(3,0,3,1);// combination?
                comDes.block(3,0,3,1)  = loco_pln->desiredcomvel(eachagentnumber);
                
                fDes = optu.block(12,0,12,1);
            }
            loco_obj->setDesired(comDes, fDes);
            loco_obj->calcTau(jpos,jvel,rotArr,force,loco_kind,controlTick_agent[eachagentnumber]);
        }

        tau = loco_obj->getTorque();
    }
    
    const int* contactMat0 = loco_obj->getConDes();
    const int* contactMat1 = loco_obj_opposite->getConDes();
    if(eachagentnumber == 0){
        memcpy(conIndDes,                 contactMat0, 4*sizeof(int));
        memcpy(conIndDes_opposite,                 contactMat1, 4*sizeof(int));
    }else if(eachagentnumber ==1){
        memcpy(conIndDes,                 contactMat1, 4*sizeof(int));
        memcpy(conIndDes_opposite,                 contactMat0, 4*sizeof(int));
    }

    jointTorqueFF = Eigen::Map< Eigen::Matrix<double,18,1> >(tau,18);
    jointTorqueFF.block(0,0,6,1).setZero();

    // Set the desired torques
    A1[eachagentnumber]->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    A1[eachagentnumber]->setGeneralizedForce(jointTorqueFF);
};

int main(int argc, char *argv[]) {
    // ============================================================ //
    // =================== SETUP RAISIM/VISUALS =================== //
    // ============================================================ //
    /// create raisim world
    raisim::World::setActivationKey(raisim::loadResource("/../../activation.raisim"));
    raisim::World world;
    world.setTimeStep(simfreq_raisim);

    raisim::OgreVis *vis = raisim::OgreVis::get();

    /// these method must be called before initApp
    vis->setWorld(&world);
    vis->setWindowSize(1792, 1200); // Should be evenly divisible by 16!!
    vis->setImguiSetupCallback(imguiSetupCallback); // These 2 lines make the interactable gui visible
    vis->setImguiRenderCallback(imguiRenderCallBack);
    vis->setKeyboardCallback(raisimKeyboardCallback);
    vis->setSetUpCallback(setupCallback);
    vis->setAntiAliasing(2);

    /// starts visualizer thread
    vis->initApp();

    /// create raisim objects
    raisim::TerrainProperties terrainProperties;
    terrainProperties.frequency = 0.0;
    terrainProperties.zScale = 0.0;
    terrainProperties.xSize = 300.0;
    terrainProperties.ySize = 300.0;
    terrainProperties.xSamples = 50;
    terrainProperties.ySamples = 50;
    terrainProperties.fractalOctaves = 0;
    terrainProperties.fractalLacunarity = 0.0;
    terrainProperties.fractalGain = 0.0;

    raisim::HeightMap *ground = world.addHeightMap(0.0, 0.0, terrainProperties);

    //vis->createGraphicalObject(ground, "terrain", "checkerboard_green");
    vis->createGraphicalObject(ground, "terrain", "collection_checkerboard_skyblue");
    world.setDefaultMaterial(0.8, 0.0, 0.0); //surface friction could be 0.8 or 1.0

    /// create raisim objects
    //double scale = 1.15;
    vis->addVisualObject("extForceArrow0", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);
    vis->addVisualObject("extForceArrow1", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);

    auto& list = vis->getVisualObjectList();

    // ============================================= //
    // === Number of agent and initial pos setup === //
    // ============================================= //
    const size_t totalagentnum = 2;
    bool constraint = true; // this sim is using constraint from raisim: it has some compliance which makes result less precise
    bool payload = false;
    bool forcedisturb = false;
    bool collaboration = true;
    bool supervised = true;
    bool distributed = false;

    // =========================================== //
    Eigen::MatrixXd agentinitial(totalagentnum, 3); // x, y, yawangle
    agentinitial.setZero(totalagentnum, 3);

    // ========= one-step delay stack ============= //
    //especially for distributed control with communication delay
    bool communicationdelay = false;
    size_t delaytick = 1;
    Eigen::MatrixXd stateDelaystack(4*totalagentnum, delaytick+1);
    stateDelaystack.setZero(4*totalagentnum, delaytick+1);
    
    if(totalagentnum == 1){
        agentinitial(0,0) = 0.0;
        agentinitial(0,1) = 0.0;
        agentinitial(0,2) = 0.0; //agent0 initial yaw angle
    }

    if(totalagentnum == 2){
        agentinitial(0,0) = 0.0;
        agentinitial(0,1) = 0.0;
        agentinitial(0,2) = 0.0; //agent0 initial yaw angle
        
        agentinitial(1,0) = 0.0;//1/sqrt(2);//0.0;
        agentinitial(1,1) = -1.0;//1/sqrt(2);//1.0;
        agentinitial(1,2) = 0.0; //agent1 initial yaw angle

        for(size_t i=0; i<delaytick; i++){
            stateDelaystack(0,i)=agentinitial(0,0);
            stateDelaystack(2,i)=agentinitial(0,1);
            stateDelaystack(4,i)=agentinitial(1,0);
            stateDelaystack(6,i)=agentinitial(1,1);
        }
    }
    // =========================================== //

    // ============================================================ //
    // ======================= SETUP Robot ======================== //
    // ============================================================ //
    std::vector<raisim::ArticulatedSystem*> A1;
    for(size_t i=0; i<totalagentnum; i++){
        double qw, qx, qy, qz;
        qw = 1.0; qx = 0.0; qy = 0.0; qz = 0.0;
        //toQuartonian(qw, qx, qy, qz, 0.0, 0.0, agentinitial(i,2));
        if(totalagentnum == 1){
            A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified_new.urdf")));
        }
        if(totalagentnum == 2){
            if(i == 0){
                A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified_new_ballj_l.urdf")));
            }
            if(i == 1){
                A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified_new_ballj_r.urdf")));
            }
        }
        vis->createGraphicalObject(A1.back(), "A1" + std::to_string(i));
        A1.back()->setGeneralizedCoordinate({agentinitial(i,0), agentinitial(i,1), 0.12, qw, qx, qy, qz,
                                            0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6});
        A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
        A1.back()->setName("A1_Robot" + std::to_string(i));
        assignBodyColors(vis, "A1", "collision", "my_gray");
    }

    if(totalagentnum == 2 && constraint == true){
        double distance_between;
        distance_between = sqrt(pow((agentinitial(0,0)-agentinitial(1,0)),2)+pow((agentinitial(0,1)-agentinitial(1,1)),2));

        //simple wire constraint between arm or between body center
        if(payload == false){
            auto wire3 = world.addStiffWire(A1[0], 0, {0,0,0.15}, A1[1], 0, {0,0,0.15}, distance_between); //connect body directly
            wire3->setStretchType(raisim::LengthConstraint::StretchType::BOTH);
            vis->createGraphicalObject(wire3, "wire3", "green");
        }
        else if(payload == true){
            auto box1 = world.addBox(0.1,distance_between-0.2,0.1, 10);
            box1->setPosition(0.5*(agentinitial(1,0)+agentinitial(0,0)),0.5*(agentinitial(1,1)+agentinitial(0,1)),0.26);//payload: 3kg w/ steplength:0.04, (0,1)
            double qw; double qx; double qy; double qz;
            toQuartonian(qw, qx, qy, qz, 0, 0, PI/2+atan2((agentinitial(1,1)-agentinitial(0,1)),(agentinitial(1,0)-agentinitial(0,0))));
            box1->setOrientation(qw, qx, qy, qz);
            auto wire10 = world.addStiffWire(A1[0], 0, {0,0,0.15}, box1, 0, {0,0.5*distance_between,0}, 0.001);
            auto wire11 = world.addStiffWire(A1[1], 0, {0,0,0.15}, box1, 0, {0,-0.5*distance_between,0}, 0.001);
            vis->createGraphicalObject(box1, "box1", "green");
            vis->createGraphicalObject(wire10, "wire10", "blue");
            vis->createGraphicalObject(wire11, "wire11", "blue");
        }
    }

    LocomotionPlanner* loco_pln0 = new LocomotionPlanner;
    LocomotionPlanner* loco_pln1 = new LocomotionPlanner;
    LocoWrapper* loco_obj0 = new LocoWrapper(argc,argv);
    LocoWrapper* loco_obj1 = new LocoWrapper(argc,argv);

    if(totalagentnum == 2){
        loco_pln0->agentidsetup(2,0);
        loco_pln1->agentidsetup(2,1);

        loco_obj0->setAgnum(0);
        loco_obj1->setAgnum(1);
    }

    // ============================================================ //
    // ================= VIEW AND RECORDING OPTIONS =============== //
    // ============================================================ //
    raisim::gui::showContacts = false;
    raisim::gui::showForces = false;
    raisim::gui::showCollision = false;
    raisim::gui::showBodies = true;

    std::string cameraview = "manual";
    bool cameraPosOriPrintout = false;
    bool panX = false;                // Pan view with robot during walking (X direction)
    bool panY = false;                // Pan view with robot during walking (Y direction)
    bool showMPC = false;              // Show the MPC body?
    bool record = false;             // Record?
    double startTime = 0*ctrlHz;    // Recording start time
    double simlength = 360*ctrlHz;   // Sim end time
    double fps = 30;            
    std::string directory = "/home/seop/";
    std::string filename  = "nothing";

    // ============================================================ //
    // ========================= VIEW SETUP ======================= //
    // ============================================================ //
    // NOTE: Pi is defined in /dynamics/dynamicsSupportFunctions.h
    // NOTE: This section still needs some work. 
    if(cameraview == "iso"){
        // vis->getCameraMan()->getCamera()->setPosition(3, 3, 2);
        vis->getCameraMan()->getCamera()->setPosition(2, -1, 0.5);
        vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(5*Pi/6-Pi/2));
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
    }else if(cameraview == "isoside"){
        //vis->getCameraMan()->getCamera()->setPosition(1.1, -2, 0.5);
        vis->getCameraMan()->getCamera()->setPosition(1.1, -3, 0.5);
        vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(4*Pi/6-Pi/2));
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
    }else if(cameraview == "side"){
        vis->getCameraMan()->getCamera()->setPosition(0, -2, 0.5);
        vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(0));
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
    }else if(cameraview == "front"){
        vis->getCameraMan()->getCamera()->setPosition(2, 0, 0.5);
        vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(Pi/2));
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
    }else if(cameraview == "top"){
        vis->getCameraMan()->getCamera()->setPosition(1, -3, 2.5);
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(1.0));
    }else if(cameraview =="manual"){
        // use the vecter3 and quaternion values printed out in the terminal
        vis->getCameraMan()->getCamera()->setPosition(3.22528, 1.33189, 2.02304);
        vis->getCameraMan()->getCamera()->setOrientation(0.45918, 0.249315, 0.406845, 0.749315);
    }
    else{
        vis->getCameraMan()->getCamera()->setPosition(1, -3, 2.5);
        vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(1.0));
    }
    unsigned long mask = 0;
    if(raisim::gui::showBodies) mask |= raisim::OgreVis::RAISIM_OBJECT_GROUP;
    if(raisim::gui::showCollision) mask |= raisim::OgreVis::RAISIM_COLLISION_BODY_GROUP;
    if(raisim::gui::showContacts) mask |= raisim::OgreVis::RAISIM_CONTACT_POINT_GROUP;
    if(raisim::gui::showForces) mask |= raisim::OgreVis::RAISIM_CONTACT_FORCE_GROUP;
    vis->setVisibilityMask(mask);

    raisim::gui::panViewX = panX;
    raisim::gui::panViewY = panY;
    raisim::gui::showMPC = showMPC;
    
    // ============================================================ //
    // ========================== RUN SIM ========================= //
    // ============================================================ //
    const std::string name = directory+filename+"_"+cameraview+".mp4";
    vis->setDesiredFPS(fps);
    long simcounter = 0;
    static bool added = false;
    Eigen::Matrix<double,  3, 3> rotMPC;
    Eigen::Matrix<double,12,1> traj = Eigen::Matrix<double, 12, 1>::Zero(12, 1);
    while (!vis->getRoot()->endRenderingQueued() && simcounter <= simlength){

        if(totalagentnum == 1){
            controller(A1,1,0, loco_pln0, loco_obj0,agentinitial);
        }
        if(totalagentnum == 2){
            controller2(A1,2,0, loco_pln0, loco_obj0, loco_obj1, agentinitial);
            controller2(A1,2,1, loco_pln1, loco_obj1, loco_obj0, agentinitial);
        }
          
        // =================================================== //
        // =========== External force application ============ //
        // =================================================== //
        if(forcedisturb){
            disturbance(A1, 0, list, "extForceArrow0", 1*ctrlHz, 50*ctrlHz, simcounter);
            if(totalagentnum == 2)
                disturbance(A1, 1, list, "extForceArrow1", 1*ctrlHz, 50*ctrlHz, simcounter);
        }


        world.integrate();        
        
        if (simcounter%30 == 0)
            vis->renderOneFrame();
        
        if (!vis->isRecording() & record & simcounter>=startTime)
            vis->startRecordingVideo(name);

        // ========================================== //
        // ========== camera purpose ================ //
        // ========================================== //
        Eigen::VectorXd jointPosTotal(18 + 1);
        Eigen::VectorXd jointVelTotal(18);
        jointPosTotal.setZero();
        jointVelTotal.setZero();
        //A1.back()->getState(jointPosTotal, jointVelTotal);
        A1[0]->getState(jointPosTotal, jointVelTotal);
        
        auto currentPos = vis->getCameraMan()->getCamera()->getPosition();
        if (raisim::gui::panViewX){
            if (cameraview=="front"){
                currentPos[0] = jointPosTotal(0)+2;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } 
            else if(cameraview=="side"){
                currentPos[0] = jointPosTotal(0);
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } 
            else if(cameraview=="iso"){
                currentPos[0] = jointPosTotal(0)+2;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } 
            else if(cameraview=="isoside"){
                currentPos[0] = jointPosTotal(0)+1.1;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            }
            else if(cameraview=="manual"){
                currentPos[0] = jointPosTotal(0)+3;
                //currentPos[1] = jointPosTotal(1)+1;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            }
        }
        if (raisim::gui::panViewY){
            if (cameraview=="front"){
                currentPos[1] = jointPosTotal(1);
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } else if(cameraview=="side"){
                currentPos[1] = jointPosTotal(1)-2;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } else if(cameraview=="iso"){
                currentPos[1] = jointPosTotal(1)-1;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            } else if(cameraview=="isoside"){
                currentPos[1] = jointPosTotal(1)-2;
                vis->getCameraMan()->getCamera()->setPosition(currentPos);
            }
        }

        simcounter++;
    }

    // End recording if still recording
    if (vis->isRecording())
        vis->stopRecordingVideoAndSave();

    /// terminate the app
    vis->closeApp();

    delete loco_obj0;
    delete loco_obj1;
    delete loco_pln0;
    delete loco_pln1;

    return 0;
}
