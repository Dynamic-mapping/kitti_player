#pragma once

#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/locale.hpp>
#include <boost/program_options.hpp>
#include <boost/progress.hpp>
#include <boost/tokenizer.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#include <std_msgs/Bool.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <time.h>

#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/crop_box.h>

#include <iostream>
#include <fstream>

// GRID_MAP
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "tracklets.h"

using namespace pcl;
using namespace std;
using namespace tf;

namespace po = boost::program_options;

bool waitSynch = false; /// Synch mode variable, refs #600

struct kitti_player_options
{
    string  path;
    float   frequency;        // publisher frequency. 1 > Kitti default 10Hz
    bool    all_data;         // publish everything
    bool    velodyne;         // publish velodyne point clouds /as PCL
    bool    gps;              // publish GPS sensor_msgs/NavSatFix    message
    bool    imu;              // publish IMU sensor_msgs/Imu Message  message
    bool    grayscale;        // publish
    bool    color;            // publish
    bool    viewer;           // enable CV viewer
    bool    timestamps;       // use KITTI timestamps;
    bool    sendTransform;    // publish velodyne TF IMU 3DOF orientation wrt fixed frame
    bool    stereoDisp;       // use precalculated stereoDisparities
    bool    viewDisparities;  // view use precalculated stereoDisparities
    bool    synchMode;        // start with synchMode on (wait for message to send next frame)
    unsigned int startFrame;  // start the replay at frame ...
    string  gpsReferenceFrame; // publish GPS points into RVIZ as RVIZ Markers
    string  object;            // the object to be select.
};

/**
 * @brief synchCallback
 * @param msg (boolean)
 *
 * if a TRUE message is received, TRUE is interpreted as "publish a new frame".
 * Then waitSynh variable is set to FALSE, and an iteration of the KittiPlayer
 * main loop is executed.
 */
void synchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO_STREAM("Synch received");
    if (msg->data)
        waitSynch = false;
}

/**
 * @brief publish_velodyne
 * @param pub The ROS publisher as reference
 * @param bird_pub The bird eye view of the front area
 * @param truth_pub The ground truth of the map
 * @param object_n The name of object to be selected
 * @param infile file with data to publish
 * @param filename_result the filename to store the results
 * @param header Header to use to publish the message
 * @return 1 if file is correctly readed, 0 otherwise
 */
int publish_velodyne(ros::Publisher &pub, ros::Publisher &bird_pub, ros::Publisher &truth_pub, string infile, string object_n, unsigned int frameid,
                     string dir_root, string filename_result, std_msgs::Header *header)
{
    fstream input(infile.c_str(), ios::in | ios::binary);
    if (!input.good())
    {
        ROS_ERROR_STREAM ( "Could not read file: " << infile );
        return 0;
    }
    else
    {
        ROS_DEBUG_STREAM ("reading " << infile);
        input.seekg(0, ios::beg);

        pcl::PointCloud<PointXYZI>::Ptr outcloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

        int i;
        for (i = 0; input.good() && !input.eof(); i++)
        {
            pcl::PointXYZI point;
            input.read((char *) &point.x, 3 * sizeof(float));
            input.read((char *) &point.intensity, sizeof(float));
            points->push_back(point);
        }
        input.close();


        /* Label type
        - 'Car'
        - 'Van'
        - 'Truck'
        - 'Pedestrian'
        - 'Person (sitting)'
        - 'Cyclist'
        - 'Tram'
        - 'Misc'
        */

        pcl::PointCloud<pcl::PointXYZI>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZI>);


        //add the tracklet labels
        {
            string trackletfile = dir_root + "/tracklet_labels.xml";
            Tracklets *tracklets = new Tracklets();
            if (!tracklets->loadFromFile(trackletfile)){
                cerr << "Could not read tracklets file: " << trackletfile << endl;
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

            //TODO select the object to show
            string objtype = object_n;
            //For each tracklet, extract the points
            for(int i = 0; i < tracklets->numberOfTracklets(); i++)
            {
                if(!tracklets->isActive(i, frameid))
                {
                    continue;
                }
                Tracklets::tTracklet* tracklet = tracklets->getTracklet(i);
                if(objtype.empty() || tracklet->objectType == objtype)
                {
                    Tracklets::tPose *pose;
                    if(tracklets->getPose(i, frameid, pose))
                    {
                        cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
                        for (size_t j = 0; j < 5000; ++j)
                        {
                          pcl::PointXYZI point;
                          point.x = 5 * rand () / (RAND_MAX + 1.0f);
                          point.y = 5 * rand () / (RAND_MAX + 1.0f);
                          point.z = 5 * rand () / (RAND_MAX + 1.0f);

                          if(fabs(point.x) <= tracklet->l/2 &&
                             fabs(point.y) <= tracklet->w/2 &&
                             fabs(point.z - tracklet->h/2) <= tracklet->h/2)
                              cloud->push_back(point);
                        }

                        Eigen::Affine3f initial_transform = Eigen::Affine3f::Identity();
                        initial_transform.translation() << pose->tx, pose->ty, pose->tz;
                        initial_transform.rotate(Eigen::AngleAxisf (pose->rx, Eigen::Vector3f::UnitX()));
                        initial_transform.rotate(Eigen::AngleAxisf (pose->ry, Eigen::Vector3f::UnitY()));
                        initial_transform.rotate(Eigen::AngleAxisf (pose->rz, Eigen::Vector3f::UnitZ()));

                        pcl::transformPointCloud(*cloud, *cloud, initial_transform);

                        *object_cloud += *cloud;
//                        *points +=  *cloud;
                    }
                }

            }
            delete tracklets;
        }

        grid_map::GridMap map({"map"});
        map.setFrameId("map");
        map.setGeometry(grid_map::Length(30, 30), 0.02, grid_map::Position(20, 0.0));
        map.add("truth", 0.0);

        for (unsigned int i = 0; i < points->size(); ++i)
        {
          auto& point = points->points[i];

          grid_map::Index index;
          grid_map::Position position(point.x, point.y);
          if (!map.getIndex(position, index)) continue;
          map.at("map", index) = point.z;
        }

        for (unsigned int i = 0; i < object_cloud->size(); ++i)
        {
          auto& point = object_cloud->points[i];

          grid_map::Index index;
          grid_map::Position position(point.x, point.y);
          if (!map.getIndex(position, index)) continue;
          map.at("truth", index) = 1.0;
        }

        sensor_msgs::Image outimg;
        grid_map::GridMapRosConverter::toImage(map, "map",
                                               sensor_msgs::image_encodings::MONO8,
                                               0,
                                               3,
                                               outimg);
        outimg.header.stamp = header->stamp;
        outimg.header.frame_id = "birdview";
        bird_pub.publish(outimg);
        cv_bridge::CvImage bird_img;

        grid_map::GridMapRosConverter::toCvImage(map, "map",
                                                sensor_msgs::image_encodings::MONO8,
                                                0, 3,
                                                bird_img);
        cv::imwrite(filename_result+"1.jpg", bird_img.image);


        grid_map::GridMapRosConverter::toImage(map, "truth",
                                               sensor_msgs::image_encodings::MONO8,
                                               0,
                                               0.5,
                                               outimg);
        outimg.header.stamp = header->stamp;
        outimg.header.frame_id = "truth";
        truth_pub.publish(outimg);
        cv_bridge::CvImage truth_img;
        grid_map::GridMapRosConverter::toCvImage(map, "truth",
                                                sensor_msgs::image_encodings::MONO8,
                                                0, 0.5,
                                                truth_img);
        cv::imwrite(filename_result+"0.jpg", truth_img.image);



        //workaround for the PCL headers... http://wiki.ros.org/hydro/Migration#PCL
        sensor_msgs::PointCloud2 pc2;

        pc2.header.frame_id = "base_link"; //ros::this_node::getName();
        pc2.header.stamp = header->stamp;
        points->header = pcl_conversions::toPCL(pc2.header);
        pub.publish(points);

        return 1;
    }
}

/**
 * @brief getCalibration
 * @param dir_root
 * @param camera_name
 * @param K double K[9]  - Calibration Matrix
 * @param D double D[5]  - Distortion Coefficients
 * @param R double R[9]  - Rectification Matrix
 * @param P double P[12] - Projection Matrix Rectified (u,v,w) = P * R * (x,y,z,q)
 * @return 1: file found, 0: file not found
 *
 *  from: http://kitti.is.tue.mpg.de/kitti/devkit_raw_data.zip
 *  calib_cam_to_cam.txt: Camera-to-camera calibration
 *
 *    - S_xx: 1x2 size of image xx before rectification
 *    - K_xx: 3x3 calibration matrix of camera xx before rectification
 *    - D_xx: 1x5 distortion vector of camera xx before rectification
 *    - R_xx: 3x3 rotation matrix of camera xx (extrinsic)
 *    - T_xx: 3x1 translation vector of camera xx (extrinsic)
 *    - S_rect_xx: 1x2 size of image xx after rectification
 *    - R_rect_xx: 3x3 rectifying rotation to make image planes co-planar
 *    - P_rect_xx: 3x4 projection matrix after rectification
 */
int getCalibration(string dir_root, string camera_name, double* K, std::vector<double> & D, double *R, double* P)
{

    string calib_cam_to_cam = dir_root + "../calib_cam_to_cam.txt";
    ifstream file_c2c(calib_cam_to_cam.c_str());
    if (!file_c2c.is_open())
        return false;

    ROS_INFO_STREAM("Reading camera" << camera_name << " calibration from " << calib_cam_to_cam);

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep {" "};

    string line = "";
    unsigned char index = 0;
    tokenizer::iterator token_iterator;

    while (getline(file_c2c, line))
    {
        // Parse string phase 1, tokenize it using Boost.
        tokenizer tok(line, sep);

        // Move the iterator at the beginning of the tokenize vector and check for K/D/R/P matrices.
        token_iterator = tok.begin();
        if (strcmp((*token_iterator).c_str(), ((string)(string("K_") + camera_name + string(":"))).c_str()) == 0) //Calibration Matrix
        {
            index = 0; //should be 9 at the end
            ROS_DEBUG_STREAM("K_" << camera_name);
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
                //std::cout << *token_iterator << '\n';
                K[index++] = boost::lexical_cast<double>(*token_iterator);
            }
        }

        // EXPERIMENTAL: use with unrectified images

        //        token_iterator=tok.begin();
        //        if (strcmp((*token_iterator).c_str(),((string)(string("D_")+camera_name+string(":"))).c_str())==0) //Distortion Coefficients
        //        {
        //            index=0; //should be 5 at the end
        //            ROS_DEBUG_STREAM("D_" << camera_name);
        //            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
        //            {
        ////                std::cout << *token_iterator << '\n';
        //                D[index++]=boost::lexical_cast<double>(*token_iterator);
        //            }
        //        }

        token_iterator = tok.begin();
        if (strcmp((*token_iterator).c_str(), ((string)(string("R_") + camera_name + string(":"))).c_str()) == 0) //Rectification Matrix
        {
            index = 0; //should be 12 at the end
            ROS_DEBUG_STREAM("R_" << camera_name);
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
                //std::cout << *token_iterator << '\n';
                R[index++] = boost::lexical_cast<double>(*token_iterator);
            }
        }

        token_iterator = tok.begin();
        if (strcmp((*token_iterator).c_str(), ((string)(string("P_rect_") + camera_name + string(":"))).c_str()) == 0) //Projection Matrix Rectified
        {
            index = 0; //should be 12 at the end
            ROS_DEBUG_STREAM("P_rect_" << camera_name);
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
                //std::cout << *token_iterator << '\n';
                P[index++] = boost::lexical_cast<double>(*token_iterator);
            }
        }

    }
    ROS_INFO_STREAM("... ok");
    return true;
}



int getGPS(string filename, sensor_msgs::NavSatFix *ros_msgGpsFix, std_msgs::Header *header)
{
    ifstream file_oxts(filename.c_str());
    if (!file_oxts.is_open())
    {
        //ROS_ERROR_STREAM("Fail to open " << filename);
        return 0;
    }

    ROS_DEBUG_STREAM("Reading GPS data from oxts file: " << filename );

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep {" "};

    string line = "";

    getline(file_oxts, line);
    tokenizer tok(line, sep);
    vector<string> s(tok.begin(), tok.end());

    ros_msgGpsFix->header.frame_id = ros::this_node::getName();
    ros_msgGpsFix->header.stamp = header->stamp;

    ros_msgGpsFix->latitude  = boost::lexical_cast<double>(s[0]);
    ros_msgGpsFix->longitude = boost::lexical_cast<double>(s[1]);
    ros_msgGpsFix->altitude  = boost::lexical_cast<double>(s[2]);

    ros_msgGpsFix->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    for (int i = 0; i < 9; i++)
        ros_msgGpsFix->position_covariance[i] = 0.0f;

    ros_msgGpsFix->position_covariance[0] = boost::lexical_cast<double>(s[23]);
    ros_msgGpsFix->position_covariance[4] = boost::lexical_cast<double>(s[23]);
    ros_msgGpsFix->position_covariance[8] = boost::lexical_cast<double>(s[23]);

    ros_msgGpsFix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    ros_msgGpsFix->status.status  = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;

    return 1;
}

int getIMU(string filename, sensor_msgs::Imu *ros_msgImu, std_msgs::Header *header)
{
    ifstream file_oxts(filename.c_str());
    if (!file_oxts.is_open())
    {
        //ROS_ERROR_STREAM("Fail to open " << filename);
        return 0;
    }

    ROS_DEBUG_STREAM("Reading IMU data from oxts file: " << filename );

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep {" "};

    string line = "";

    getline(file_oxts, line);
    tokenizer tok(line, sep);
    vector<string> s(tok.begin(), tok.end());

    ros_msgImu->header.frame_id = ros::this_node::getName();
    ros_msgImu->header.stamp = header->stamp;

    //    - ax:      acceleration in x, i.e. in direction of vehicle front (m/s^2)
    //    - ay:      acceleration in y, i.e. in direction of vehicle left (m/s^2)
    //    - az:      acceleration in z, i.e. in direction of vehicle top (m/s^2)
    ros_msgImu->linear_acceleration.x = boost::lexical_cast<double>(s[11]);
    ros_msgImu->linear_acceleration.y = boost::lexical_cast<double>(s[12]);
    ros_msgImu->linear_acceleration.z = boost::lexical_cast<double>(s[13]);

    //    - vf:      forward velocity, i.e. parallel to earth-surface (m/s)
    //    - vl:      leftward velocity, i.e. parallel to earth-surface (m/s)
    //    - vu:      upward velocity, i.e. perpendicular to earth-surface (m/s)
    ros_msgImu->angular_velocity.x = boost::lexical_cast<double>(s[8]);
    ros_msgImu->angular_velocity.y = boost::lexical_cast<double>(s[9]);
    ros_msgImu->angular_velocity.z = boost::lexical_cast<double>(s[10]);

    //    - roll:    roll angle (rad),  0 = level, positive = left side up (-pi..pi)
    //    - pitch:   pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
    //    - yaw:     heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
    tf::Quaternion q = tf::createQuaternionFromRPY(   boost::lexical_cast<double>(s[3]),
                                                      boost::lexical_cast<double>(s[4]),
                                                      boost::lexical_cast<double>(s[5])
                                                  );
    ros_msgImu->orientation.x = q.getX();
    ros_msgImu->orientation.y = q.getY();
    ros_msgImu->orientation.z = q.getZ();
    ros_msgImu->orientation.w = q.getW();

    return 1;
}




/// Cartesian coordinates struct, refs# 522
struct Xy
{
    double x;
    double y;
};

/** Conversion between geographic and UTM coordinates
    Adapted from:  http://www.uwgb.edu/dutchs/UsefulData/ConvertUTMNoOZ.HTM
    Refs# 522
**/
Xy latlon2xy_helper(double lat, double lngd)
{
    // WGS 84 datum
    double eqRad = 6378137.0;
    double flat = 298.2572236;

    // constants used in calculations:
    double a = eqRad;           // equatorial radius in meters
    double f = 1.0 / flat;        // polar flattening
    double b = a * (1.0 - f);     // polar radius
    double e = sqrt(1.0 - (pow(b, 2) / pow(a, 2))); // eccentricity
    double k0 = 0.9996;
    double drad = M_PI / 180.0;

    double phi = lat * drad;   // convert latitude to radians
    double utmz = 1.0 + floor((lngd + 180.0) / 6.0); // longitude to utm zone
    double zcm = 3.0 + 6.0 * (utmz - 1.0) - 180.0;     // central meridian of a zone
    double esq = (1.0 - (b / a) * (b / a));
    double e0sq = e * e / (1.0 - e * e);
    double M = 0.0;
    double M0 = 0.0;
    double N = a / sqrt(1.0 - pow(e * sin(phi), 2));
    double T = pow(tan(phi), 2);
    double C = e0sq * pow(cos(phi), 2);
    double A = (lngd - zcm) * drad * cos(phi);

    // calculate M (USGS style)
    M = phi * (1.0 - esq * (1.0 / 4.0 + esq * (3.0 / 64.0 + 5.0 * esq / 256.0)));
    M = M - sin(2.0 * phi) * (esq * (3.0 / 8.0 + esq * (3.0 / 32.0 + 45.0 * esq / 1024.0)));
    M = M + sin(4.0 * phi) * (esq * esq * (15.0 / 256.0 + esq * 45.0 / 1024.0));
    M = M - sin(6.0 * phi) * (esq * esq * esq * (35.0 / 3072.0));
    M = M * a; // Arc length along standard meridian

    // now we are ready to calculate the UTM values...
    // first the easting (relative to CM)
    double x = k0 * N * A * (1.0 + A * A * ((1.0 - T + C) / 6.0 + A * A * (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * e0sq) / 120.0));
    x = x + 500000.0; // standard easting

    // now the northing (from the equator)
    double y = k0 * (M - M0 + N * tan(phi) * (A * A * (1.0 / 2.0 + A * A * ((5.0 - T + 9.0 * C + 4.0 * C * C) / 24.0 + A * A * (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * e0sq) / 720.0))));
    if (y < 0)
    {
        y = 10000000.0 + y; // add in false northing if south of the equator
    }
    double easting  = x;
    double northing = y;

    Xy coords;
    coords.x = easting;
    coords.y = northing;

    return coords;
}


/**
 * @brief parseTime
 * @param timestamp in Epoch
 * @return std_msgs::Header with input timpestamp converted from file input
 *
 * Epoch time conversion
 * http://www.epochconverter.com/programming/functions-c.php
 */
std_msgs::Header parseTime(string timestamp)
{

    std_msgs::Header header;

    // example: 2011-09-26 13:21:35.134391552
    //          01234567891111111111222222222
    //                    0123456789012345678
    struct tm t = {0};  // Initalize to all 0's
    t.tm_year = boost::lexical_cast<int>(timestamp.substr(0, 4)) - 1900;
    t.tm_mon  = boost::lexical_cast<int>(timestamp.substr(5, 2)) - 1;
    t.tm_mday = boost::lexical_cast<int>(timestamp.substr(8, 2));
    t.tm_hour = boost::lexical_cast<int>(timestamp.substr(11, 2));
    t.tm_min  = boost::lexical_cast<int>(timestamp.substr(14, 2));
    t.tm_sec  = boost::lexical_cast<int>(timestamp.substr(17, 2));
    t.tm_isdst = -1;
    time_t timeSinceEpoch = mktime(&t);

    header.stamp.sec  = timeSinceEpoch;
    header.stamp.nsec = boost::lexical_cast<int>(timestamp.substr(20, 8));

    return header;
}
