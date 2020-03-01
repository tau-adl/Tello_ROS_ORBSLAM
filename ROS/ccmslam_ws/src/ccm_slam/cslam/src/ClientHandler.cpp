/**
* This file is part of CCM-SLAM.
*
* Copyright (C): Patrik Schmuck <pschmuck at ethz dot ch> (ETH Zurich)
* For more information see <https://github.com/patriksc/CCM-SLAM>
*
* CCM-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* CCM-SLAM is based in the monocular version of ORB-SLAM2 by Raúl Mur-Artal.
* CCM-SLAM partially re-uses modules of ORB-SLAM2 in modified or unmodified condition.
* For more information see <https://github.com/raulmur/ORB_SLAM2>.
*
* CCM-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with CCM-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <cslam/ClientHandler.h>

namespace cslam {

ClientHandler::ClientHandler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, vocptr pVoc, dbptr pDB, mapptr pMap, size_t ClientId, uidptr pUID, eSystemState SysState, const string &strCamFile, viewptr pViewer)
    : mpVoc(pVoc),mpKFDB(pDB),mpMap(pMap),
      mNh(Nh),mNhPrivate(NhPrivate),
      mClientId(ClientId), mpUID(pUID), mSysState(SysState),
      mstrCamFile(strCamFile),
      mpViewer(pViewer),mbReset(false)
{
    if(mpVoc == nullptr || mpKFDB == nullptr || mpMap == nullptr || (mpUID == nullptr && mSysState == eSystemState::SERVER))
    {
        cout << ("In \" ClientHandler::ClientHandler(...)\": nullptr exception") << endl;
        throw estd::infrastructure_ex();
    }

    mpMap->msuAssClients.insert(mClientId);

    mg2oS_wcurmap_wclientmap = g2o::Sim3(); //identity transformation

    if(mSysState == eSystemState::CLIENT)
    {
        std::string TopicNameCamSub;

        mNhPrivate.param("TopicNameCamSub",TopicNameCamSub,string("nospec"));
        mSubCam = mNh.subscribe<sensor_msgs::Image>(TopicNameCamSub,10,boost::bind(&ClientHandler::CamImgCb,this,_1));

        cout << "Camera Input topic: " << TopicNameCamSub << endl;
    }
}
#ifdef LOGGING
void ClientHandler::InitializeThreads(boost::shared_ptr<estd::mylog> pLogger)
#else
void ClientHandler::InitializeThreads()
#endif
{
    #ifdef LOGGING
    this->InitializeCC(pLogger);
    #else
    this->InitializeCC();
    #endif

    if(mSysState == eSystemState::CLIENT)
    {
        this->InitializeClient();
    }
    else if(mSysState == eSystemState::SERVER)
    {
        this->InitializeServer();
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::InitializeThreads(): invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }
}

#ifdef LOGGING
void ClientHandler::InitializeCC(boost::shared_ptr<mylog> pLogger)
#else
void ClientHandler::InitializeCC()
#endif
{
    std::stringstream* ss;

    mpCC.reset(new CentralControl(mNh,mNhPrivate,mClientId,mSysState,shared_from_this(),mpUID));

    if(mSysState == eSystemState::CLIENT)
    {
        ss = new stringstream;
        *ss << "FrameId";
        mNhPrivate.param(ss->str(),mpCC->mNativeOdomFrame,std::string("nospec"));
    }
    else if(mSysState == eSystemState::SERVER)
    {
        ss = new stringstream;
        *ss << "FrameId" << mClientId;
        mNhPrivate.param(ss->str(),mpCC->mNativeOdomFrame,std::string("nospec"));
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::InitializeThreads(): invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }

    if(mpCC->mNativeOdomFrame=="nospec")
    {
        ROS_ERROR_STREAM("In \" ServerCommunicator::ServerCommunicator(...)\": bad parameters");
        throw estd::infrastructure_ex();
    }

    {
        if(mSysState==CLIENT)
        {
            cv::FileStorage fSettings(mstrCamFile, cv::FileStorage::READ);

            float c0t00 = fSettings["Cam0.T00"];
            float c0t01 = fSettings["Cam0.T01"];
            float c0t02 = fSettings["Cam0.T02"];
            float c0t03 = fSettings["Cam0.T03"];
            float c0t10 = fSettings["Cam0.T10"];
            float c0t11 = fSettings["Cam0.T11"];
            float c0t12 = fSettings["Cam0.T12"];
            float c0t13 = fSettings["Cam0.T13"];
            float c0t20 = fSettings["Cam0.T20"];
            float c0t21 = fSettings["Cam0.T21"];
            float c0t22 = fSettings["Cam0.T22"];
            float c0t23 = fSettings["Cam0.T23"];
            float c0t30 = fSettings["Cam0.T30"];
            float c0t31 = fSettings["Cam0.T31"];
            float c0t32 = fSettings["Cam0.T32"];
            float c0t33 = fSettings["Cam0.T33"];
            mpCC->mT_SC << c0t00,c0t01,c0t02,c0t03,c0t10,c0t11,c0t12,c0t13,c0t20,c0t21,c0t22,c0t23,c0t30,c0t31,c0t32,c0t33;
        }
        else
        {
            //no mstrCamFile on Server...
        }
    }

    mpMap->mOdomFrame = mpCC->mNativeOdomFrame;
    mpMap->AddCCPtr(mpCC);

    #ifdef LOGGING
    mpCC->mpLogger = pLogger;
    #endif

    delete ss;
}

void ClientHandler::InitializeClient()
{
    cout << "Client " << mClientId << " --> Initialize Threads" << endl;

    //  Pose Publisher
    std::stringstream* ss;
    ss = new stringstream;
    *ss << "PoseOut" << "Client" << mClientId;
    string PubPoseTopicName = ss->str();
    mPubPose = mNh.advertise<geometry_msgs::PoseStamped>(PubPoseTopicName, 10);

    //+++++ Create Drawers. These are used by the Viewer +++++
    mpViewer.reset(new Viewer(mpMap,mpCC));
    usleep(10000);
    //+++++ Initialize the Local Mapping thread +++++
    mpMapping.reset(new LocalMapping(mpCC,mpMap,mpKFDB,mpViewer));
    usleep(10000);
//    +++++ Initialize the communication thread +++++
    mpComm.reset(new Communicator(mpCC,mpVoc,mpMap,mpKFDB));
    mpComm->SetMapping(mpMapping);
    usleep(10000);
    mpMap->SetCommunicator(mpComm);
    mpMapping->SetCommunicator(mpComm);
    usleep(10000);
    //+++++ Initialize the tracking thread +++++
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracking.reset(new Tracking(mpCC, mpVoc, mpViewer, mpMap, mpKFDB, mstrCamFile, mClientId));
    usleep(10000);
    mpTracking->SetCommunicator(mpComm);
    mpTracking->SetLocalMapper(mpMapping);
    mpViewer->SetTracker(mpTracking);
    usleep(10000);
    //Launch Threads
    //Should no do that before, a fast system might already use a pointe before it was set -> segfault
    mptMapping.reset(new thread(&LocalMapping::RunClient,mpMapping));
    mptComm.reset(new thread(&Communicator::RunClient,mpComm));
    mptViewer.reset(new thread(&Viewer::RunClient,mpViewer));
    ptrPoseStamped.reset(new thread(&ClientHandler::PublishPoseThread, this));
    usleep(10000);
}

void ClientHandler::InitializeServer()
{
    cout << "Client " << mClientId << " --> Initialize Threads" << endl;
    std::stringstream* ss;
    ss = new stringstream;
    *ss << "TransOut" << "Server" << mClientId;
    string PubTransTopicName = ss->str();

    //+++++ Initialize the Loop Finder thread and launch +++++
    mpLoopFinder.reset(new LoopFinder(mpCC,mpKFDB,mpVoc,mpMap));
    mptLoopClosure.reset(new thread(&LoopFinder::Run,mpLoopFinder));
    usleep(10000);
    mPubTrans = mNh.advertise<geometry_msgs::TransformStamped>(PubTransTopicName, 1);
    //+++++ Initialize the Local Mapping thread +++++
    mpMapping.reset(new LocalMapping(mpCC,mpMap,mpKFDB,mpViewer));
    mpMapping->SetLoopFinder(mpLoopFinder); //tempout
    usleep(10000);
    //+++++ Initialize the communication thread +++++
    mpComm.reset(new Communicator(mpCC,mpVoc,mpMap,mpKFDB));
    mpComm->SetMapping(mpMapping);
    usleep(10000);
    mpMapping->SetCommunicator(mpComm);
    mpMap->SetCommunicator(mpComm);
    usleep(10000);
    //Launch Threads
    //Should not do that before, a fast system might already use a pointer before it was set -> segfault
    mptMapping.reset(new thread(&LocalMapping::RunServer,mpMapping));
    mptComm.reset(new thread(&Communicator::RunServer,mpComm));
    ptrTransformStamped.reset(new thread(&ClientHandler::PublishTransThread, this));
    usleep(10000);
    if(mpCC->mpCH == nullptr)
    {
        ROS_ERROR_STREAM("ClientHandler::InitializeThreads()\": mpCC->mpCH is nullptr");
        throw estd::infrastructure_ex();
    }
}

void ClientHandler::ChangeMap(mapptr pMap, g2o::Sim3 g2oS_wnewmap_wcurmap)
{
    mpMap = pMap;

    mg2oS_wcurmap_wclientmap = g2oS_wnewmap_wcurmap*mg2oS_wcurmap_wclientmap;
    mpCC->mg2oS_wcurmap_wclientmap = mg2oS_wcurmap_wclientmap;

    bool bLockedComm = mpCC->LockComm(); //should be locked and therefore return false
//    #ifdef LOGGING
//    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
//    #endif
    bool bLockedMapping = mpCC->LockMapping(); //should be locked and therefore return false
//    #ifdef LOGGING
//    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
//    #endif

    if(bLockedComm || bLockedMapping)
    {
        if(bLockedComm) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::ChangeMap(): Comm not locked: " << endl;
        if(bLockedMapping) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::ChangeMap(): Mapping not locked: " << endl;
        throw infrastructure_ex();
    }

//    #ifdef LOGGING
//    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
//    #endif

    mpComm->ChangeMap(mpMap);
    mpMapping->ChangeMap(mpMap); //tempout
    mpLoopFinder->ChangeMap(mpMap); //tempout

//    #ifdef LOGGING
//    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
//    #endif
}


void ClientHandler::SetMapMatcher(matchptr pMatch)
{
    mpMapMatcher = pMatch;
    mpComm->SetMapMatcher(mpMapMatcher);
    mpMapping->SetMapMatcher(mpMapMatcher);
}

void ClientHandler::CamImgCb(sensor_msgs::ImageConstPtr pMsg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(pMsg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracking->Reset();
            mbReset = false;
        }
    }

//    position_ = mpTracking->GrabImageMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    mpTracking->GrabImageMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    if (mpTracking->lost_tracking_counter == 0) {
        receivedImageFlag = true;
    }


//    if (mpTracking->mState == mpTracking->OK) {
//        this->PublishPositionAsPoseStamped(position);
//    }

//    if (mpTracking->mState == mpTracking->OK) {
//        kfptr pRef = mpTracking->mLastFrame->mpReferenceKF;
//        if (pRef) {
//            cv::Mat position = pRef->GetPose();
//            if (!position.empty()) {
//                this->PublishPositionAsPoseStamped(position);
//            }
//        }
//    }

}

void ClientHandler::PublishPoseThread(){
    float fScale = static_cast<float>(params::vis::mfScaleFactor);
    geometry_msgs::PoseStamped pose_msg;
    tf2::Quaternion tfQuaternion;
    geometry_msgs::Quaternion quat_msg;
    cv::Mat Tcw;
    cv::Mat Rcw;
    cv::Mat tcw;
    cv::Mat ow;
    pose_msg.header.frame_id = "world";
    while(1) {
//        usleep(33333);
        usleep(3333);
        if (receivedImageFlag) {
            receivedImageFlag = false;
            if (mpTracking->mState == mpTracking->OK) {


                Tcw = mpTracking->mCurrentFrame->mTcw;
                if (Tcw.dims >= 2) {


                    Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
                    tcw = Tcw.rowRange(0,3).col(3);
                    ow = -Rcw.t()*tcw;

                    // this suppose to be roll, so new_pitch = old_roll
                    double pitch = std::atan2(Rcw.at<float>(2, 1), Rcw.at<float>(2, 2));

                    // this suppose to be pitch, so new_yaw = old_pitch
                    double yaw = std::atan2(-Rcw.at<float>(2, 0),
                            sqrt(pow(Rcw.at<float>(2, 1), 2) + pow(Rcw.at<float>(2, 2), 2)));

                    // this suppose to be yaw, so new_roll = -old_yaw
                    double roll = -std::atan2(Rcw.at<float>(1, 0), Rcw.at<float>(0, 0));


                    tfQuaternion.setRPY(roll, pitch, yaw);

                    quat_msg = tf2::toMsg(tfQuaternion);


                    pose_msg.pose.position.x = (fScale)*ow.at<float>(2);   // x = z
                    pose_msg.pose.position.y = -(fScale)*ow.at<float>(0);  // y = -x
                    pose_msg.pose.position.z = -(fScale)*ow.at<float>(1); // z = -y
                    pose_msg.header.stamp = ros::Time::now();
                    pose_msg.pose.orientation = quat_msg;
                    mPubPose.publish(pose_msg);
                }
            }
        }
    }
}

void ClientHandler::PublishTransThread(){
    tf2::Quaternion tfQuaternion;
    g2o::Sim3 g2oS_wnewmap_wcurmap;
    geometry_msgs::TransformStamped msgtf;
    cv::Mat Tcw;
    cv::Mat Rcw;
    cv::Mat tcw;
    cv::Mat ow;
    msgtf.header.frame_id = "world";
    msgtf.child_frame_id = "idk";
    double s;
    while(1) {
        usleep(3333);
        g2oS_wnewmap_wcurmap = mpCC->mg2oS_wcurmap_wclientmap;
        s = g2oS_wnewmap_wcurmap.scale();
        msgtf.transform.translation.x = g2oS_wnewmap_wcurmap.translation()[2] * s;
        msgtf.transform.translation.y = -g2oS_wnewmap_wcurmap.translation()[0] * s;
        msgtf.transform.translation.z = -g2oS_wnewmap_wcurmap.translation()[1] * s;

        // I can use this primitive Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix(); in order to get
        // back to rotation coords and correct the same as we did in the Pose Publisher.
//        msgtf.transform.rotation.w = g2oS_wnewmap_wcurmap.rotation().coeffs()[0];
//        msgtf.transform.rotation.x = g2oS_wnewmap_wcurmap.rotation().coeffs()[1];
//        msgtf.transform.rotation.y = g2oS_wnewmap_wcurmap.rotation().coeffs()[2];
//        msgtf.transform.rotation.z = g2oS_wnewmap_wcurmap.rotation().coeffs()[3];


        tf::Quaternion q(
                g2oS_wnewmap_wcurmap.rotation().coeffs()[0],
                g2oS_wnewmap_wcurmap.rotation().coeffs()[1],
                g2oS_wnewmap_wcurmap.rotation().coeffs()[2],
                g2oS_wnewmap_wcurmap.rotation().coeffs()[3]);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;

        // this suppose to be roll, so new_pitch = old_roll
        // this suppose to be pitch, so new_yaw = old_pitch
        // this suppose to be yaw, so new_roll = -old_yaw

        m.getRPY(roll, pitch, yaw);


        tfQuaternion.setRPY(-yaw, roll, pitch);


        geometry_msgs::Quaternion quat_msg = tf2::toMsg(tfQuaternion);

        msgtf.transform.rotation.w = quat_msg.w;
        msgtf.transform.rotation.x = quat_msg.x;
        msgtf.transform.rotation.y = quat_msg.y;
        msgtf.transform.rotation.z = quat_msg.z;

        msgtf.header.stamp = ros::Time::now();
        mPubTrans.publish(msgtf);
    }
}


void ClientHandler::PublishPose(cv::Mat Tcw){
    float fScale = static_cast<float>(params::vis::mfScaleFactor);
    geometry_msgs::PoseStamped pose_msg;
    tf2::Quaternion tfQuaternion;
    geometry_msgs::Quaternion quat_msg;
    cv::Mat Rcw;
    cv::Mat tcw;
    cv::Mat ow;
    pose_msg.header.frame_id = "world";


    if (Tcw.dims >= 2) {


        Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
        tcw = Tcw.rowRange(0,3).col(3);
        ow = -Rcw.t()*tcw;
        double roll = std::atan2(Rcw.at<float>(2, 1), Rcw.at<float>(2, 2));
        double yaw = std::atan2(-Rcw.at<float>(2, 0),
                                 sqrt(pow(Rcw.at<float>(2, 1), 2) + pow(Rcw.at<float>(2, 2), 2)));
        double pitch = -std::atan2(Rcw.at<float>(1, 0), Rcw.at<float>(0, 0));

        tfQuaternion.setRPY(pitch, roll, yaw);
        quat_msg = tf2::toMsg(tfQuaternion);


        pose_msg.pose.position.x = (fScale)*ow.at<float>(2);   // x = z
        pose_msg.pose.position.y = -(fScale)*ow.at<float>(0);  // y = -x
        pose_msg.pose.position.z = -(fScale)*ow.at<float>(1); // z = -y
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.orientation = quat_msg;
        mPubPose.publish(pose_msg);
    }
}


//void ClientHandler::PublishPoseThread(){
//    geometry_msgs::PoseStamped pose_msg;
//    tf2::Quaternion tfQuaternion;
//    geometry_msgs::Quaternion quat_msg;
//    cv::Mat Tcw;
//    cv::Mat Rcw;
//
//    pose_msg.header.frame_id = "world";
//    while(1) {
////        usleep(33333);
//        usleep(3333);
//        if (receivedImageFlag) {
//            receivedImageFlag = false;
//            if (mpTracking->mState == mpTracking->OK) {
//
//
//                Tcw = mpTracking->mCurrentFrame->mTcw;
//
//                if (Tcw.dims >= 2) {
//
//
//                    Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
//
//                    double pitch = std::atan2(Rcw.at<float>(2, 0), Rcw.at<float>(2, 1));
//                    double roll = std::acos(Rcw.at<float>(2, 2));
//                    double yaw = -std::atan2(Rcw.at<float>(0, 2), Rcw.at<float>(1, 2));
//
//                    tfQuaternion.setRPY(pitch, roll, yaw);
//                    quat_msg = tf2::toMsg(tfQuaternion);
//
//
//                    pose_msg.pose.position.x = mpViewer->msg_point_out.z;
//                    pose_msg.pose.position.y = -mpViewer->msg_point_out.x;
//                    pose_msg.pose.position.z = -mpViewer->msg_point_out.y;
//                    pose_msg.header.stamp = ros::Time::now();
//                    pose_msg.pose.orientation = quat_msg;
//                    mPubPose.publish(pose_msg);
//                }
//            }
//        }
//    }
//}

//void ClientHandler::PublishPositionAsPoseStamped (cv::Mat position) {
//    std::stringstream* ss;
//    tf::Transform grasp_tf = this->TransformFromMat (position);
//    current_frame_time_ = ros::Time::now();
//    ss = new stringstream;
//    *ss << "world";
//    string map_frame_id_param_ = ss->str();
//    tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time_, map_frame_id_param_);
//    geometry_msgs::PoseStamped pose_msg;
//    tf::poseStampedTFToMsg (grasp_tf_pose, pose_msg);
//    // current_frame_time_.
//    pose_msg.header.stamp = ros::Time::now();
//    mPubPose.publish(pose_msg);
//}

//tf::Transform ClientHandler::TransformFromMat (cv::Mat position_mat) {
//    cv::Mat rotation(3,3,CV_32F);
//    cv::Mat translation(3,1,CV_32F);
//
//    rotation = position_mat.rowRange(0,3).colRange(0,3);
//    translation = position_mat.rowRange(0,3).col(3);
//
//    tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
//                                      rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
//                                      rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
//    );
//
//    tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));
//
//    //Coordinate transformation matrix from orb coordinate system to ros coordinate system
//    const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
//                                       -1, 0, 0,
//                                       0,-1, 0);
//
//    //Transform from orb coordinate system to ros coordinate system on camera coordinates
//    tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
//    tf_camera_translation = tf_orb_to_ros*tf_camera_translation;
//
//    //Inverse matrix
//    tf_camera_rotation = tf_camera_rotation.transpose();
//    tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);
//
//    //Transform from orb coordinate system to ros coordinate system on map coordinates
//    tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
//    tf_camera_translation = tf_orb_to_ros*tf_camera_translation;
//
//    return tf::Transform (tf_camera_rotation, tf_camera_translation);
//}

void ClientHandler::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

ClientHandler::kfptr ClientHandler::GetCurrentRefKFfromTracking()
{
    if(mpTracking->mState < 2)
        return nullptr;
    else
        return mpTracking->GetReferenceKF();
}

int ClientHandler::GetNumKFsinLoopFinder()
{
    if(mpLoopFinder)
        return mpLoopFinder->GetNumKFsinQueue();
    else
        return -1;
}

int ClientHandler::GetNumKFsinMapMatcher()
{
    if(mpMapMatcher)
        return mpMapMatcher->GetNumKFsinQueue();
    else
        return -1;
}

void ClientHandler::ClearCovGraph(size_t MapId)
{
    mpMapping->ClearCovGraph(MapId);
}

//#ifdef LOGGING
//void ClientHandler::SetLogger(boost::shared_ptr<mylog> pLogger)
//{
//    mpCC->mpLogger = pLogger;
//}
//#endif

} //end ns
