// redmine usage: This commit refs #388 @2h

// ###############################################################################################
// ###############################################################################################
// ###############################################################################################

/*
 * KITTI_PLAYER v2.
 *
 * Augusto Luis Ballardini, ballardini@disco.unimib.it
 *
 * https://github.com/iralabdisco/kitti_player
 *
 * WARNING: this package is using some C++11
 *
 */

// ###############################################################################################
// ###############################################################################################
// ###############################################################################################
#include "kitti_player.h"

using namespace pcl;
using namespace std;
using namespace tf;

namespace po = boost::program_options;



/**
 * @brief main Kitti_player, a player for KITTI raw datasets
 * @param argc
 * @param argv
 * @return 0 and ros::shutdown at the end of the dataset, -1 if errors
 *
 * Allowed options:
 *   -h [ --help ]                       help message
 *   -d [ --directory  ] arg             *required* - path to the kitti dataset Directory
 *   -f [ --frequency  ] arg (=1)        set replay Frequency
 *   -a [ --all        ] [=arg(=1)] (=0) replay All data
 *   -v [ --velodyne   ] [=arg(=1)] (=0) replay Velodyne data
 *   -g [ --gps        ] [=arg(=1)] (=0) replay Gps data
 *   -i [ --imu        ] [=arg(=1)] (=0) replay Imu data
 *   -l [ --labels     ] [=arg(=1)] (=0) replay labels data
 *   -G [ --grayscale  ] [=arg(=1)] (=0) replay Stereo Grayscale images
 *   -C [ --color      ] [=arg(=1)] (=0) replay Stereo Color images
 *   -V [ --viewer     ] [=arg(=1)] (=0) enable image viewer
 *   -T [ --timestamps ] [=arg(=1)] (=0) use KITTI timestamps
 *   -s [ --stereoDisp ] [=arg(=1)] (=0) use pre-calculated disparities
 *   -D [ --viewDisp   ] [=arg(=1)] (=0) view loaded disparity images
 *   -F [ --frame      ] [=arg(=0)] (=0) start playing at frame ...
 *
 * Datasets can be downloaded from: http://www.cvlibs.net/datasets/kitti/raw_data.php
 */
int main(int argc, char **argv)
{
    kitti_player_options options;
    po::variables_map vm;

    po::options_description desc("Kitti_player, a player for KITTI raw datasets\nDatasets can be downloaded from: http://www.cvlibs.net/datasets/kitti/raw_data.php\n\nAllowed options", 200);
    desc.add_options()
    ("help,h"                                                                                                    ,  "help message")
    ("directory ,d",  po::value<string>       (&options.path)->required()                                        ,  "*required* - path to the kitti dataset Directory")
    ("frequency ,f",  po::value<float>        (&options.frequency)        ->default_value(1.0)                     ,  "set replay Frequency")
    ("all       ,a",  po::value<bool>         (&options.all_data)         ->default_value(0) ->implicit_value(1)   ,  "replay All data")
    ("velodyne  ,v",  po::value<bool>         (&options.velodyne)         ->default_value(0) ->implicit_value(1)   ,  "replay Velodyne data")
    ("gps       ,g",  po::value<bool>         (&options.gps)              ->default_value(0) ->implicit_value(1)   ,  "replay Gps data")
    ("imu       ,i",  po::value<bool>         (&options.imu)              ->default_value(0) ->implicit_value(1)   ,  "replay Imu data")
    ("grayscale ,G",  po::value<bool>         (&options.grayscale)        ->default_value(0) ->implicit_value(1)   ,  "replay Stereo Grayscale images")
    ("color     ,C",  po::value<bool>         (&options.color)            ->default_value(0) ->implicit_value(1)   ,  "replay Stereo Color images")
    ("viewer    ,V",  po::value<bool>         (&options.viewer)           ->default_value(0) ->implicit_value(1)   ,  "enable image viewer")
    ("timestamps,T",  po::value<bool>         (&options.timestamps)       ->default_value(0) ->implicit_value(1)   ,  "use KITTI timestamps")
    ("stereoDisp,s",  po::value<bool>         (&options.stereoDisp)       ->default_value(0) ->implicit_value(1)   ,  "use pre-calculated disparities")
    ("viewDisp  ,D ", po::value<bool>         (&options.viewDisparities)  ->default_value(0) ->implicit_value(1)   ,  "view loaded disparity images")
    ("frame     ,F",  po::value<unsigned int> (&options.startFrame)       ->default_value(0) ->implicit_value(0)   ,  "start playing at frame...")
    ("gpsPoints ,p",  po::value<string>       (&options.gpsReferenceFrame)->default_value("")                      ,  "publish GPS/RTK markers to RVIZ, having reference frame as <reference_frame> [example: -p map]")
    ("object    ,o",  po::value<string>       (&options.object)           ->default_value(" ")                     ,  "the obejct to be selected")
    ("synchMode ,S",  po::value<bool>         (&options.synchMode)        ->default_value(0) ->implicit_value(1)   ,  "Enable Synch mode (wait for signal to load next frame [std_msgs/Bool data: true]")
    ;

    try // parse options
    {
        po::parsed_options parsed = po::command_line_parser(argc - 2, argv).options(desc).allow_unregistered().run();
        po::store(parsed, vm);
        po::notify(vm);

        vector<string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);

        // Can't handle __ros (ROS parameters ... )
        //        if (to_pass_further.size()>0)
        //        {
        //            ROS_WARN_STREAM("Unknown Options Detected, shutting down node\n");
        //            cerr << desc << endl;
        //            return 1;
        //        }
    }
    catch (...)
    {
        cerr << desc << endl;

        cout << "kitti_player needs a directory tree like the following:" << endl;
        cout << "├── calib_cam_to_cam.txt      " << endl;
        cout << "└── 2011_09_26_drive_0001_sync" << endl;
        cout << "    ├── image_00              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    │   └ timestamps.txt      " << endl;
        cout << "    ├── image_01              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    │   └ timestamps.txt      " << endl;
        cout << "    ├── image_02              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    │   └ timestamps.txt      " << endl;
        cout << "    ├── image_03              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    │   └ timestamps.txt      " << endl;
        cout << "    ├── oxts                  " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    │   └ timestamps.txt      " << endl;
        cout << "    ├── velodyne_points       " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    │     └ timestamps.txt    " << endl;
        cout << "    └── tracklet_labels.xml   " << endl << endl;

        ROS_WARN_STREAM("Parse error, shutting down node\n");
        return -1;
    }

    ros::init(argc, argv, "kitti_player");
    ros::NodeHandle node("kitti_player");
    ros::Rate loop_rate(options.frequency);

    /// This sets the logger level; use this to disable all ROS prints
    if ( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) )
        ros::console::notifyLoggerLevelsChanged();
    else
        std::cout << "Error while setting the logger level!" << std::endl;

    DIR *dir;
    struct dirent *ent;
    unsigned int total_entries = 0;        //number of elements to be played
    unsigned int entries_played  = 0;      //number of elements played until now
    unsigned int len = 0;                  //counting elements support variable
    string dir_root             ;
    string dir_result           ;
    string dir_image00          ;
    string full_filename_image00;
    string dir_timestamp_image00;
    string dir_image01          ;
    string full_filename_image01;
    string dir_timestamp_image01;
    string dir_image02          ;
    string full_filename_image02;
    string dir_timestamp_image02;
    string dir_image03          ;
    string full_filename_image03;
    string dir_timestamp_image03;
    string dir_image04          ;
    string full_filename_image04;
    string dir_Disparities    ;
    string full_filename_Disparities;
    string dir_oxts             ;
    string full_filename_oxts;
    string dir_timestamp_oxts;
    string dir_velodyne_points  ;
    string full_filename_velodyne;
    string full_filename_result;
    string dir_timestamp_velodyne; //average of start&end (time of scan)
    string str_support;
    cv::Mat cv_image00;
    cv::Mat cv_image01;
    cv::Mat cv_image02;
    cv::Mat cv_image03;
    cv::Mat cv_image04;
    cv::Mat cv_disparities;
    std_msgs::Header header_support;

    image_transport::ImageTransport it(node);
    image_transport::CameraPublisher pub00 = it.advertiseCamera("grayscale/left/image_rect", 1);
    image_transport::CameraPublisher pub01 = it.advertiseCamera("grayscale/right/image_rect", 1);
    image_transport::CameraPublisher pub02 = it.advertiseCamera("color/left/image_rect", 1);
    image_transport::CameraPublisher pub03 = it.advertiseCamera("color/right/image_rect", 1);

    sensor_msgs::Image ros_msg00;
    sensor_msgs::Image ros_msg01;
    sensor_msgs::Image ros_msg02;
    sensor_msgs::Image ros_msg03;


//    sensor_msgs::CameraInfo ros_cameraInfoMsg;
    sensor_msgs::CameraInfo ros_cameraInfoMsg_camera00;
    sensor_msgs::CameraInfo ros_cameraInfoMsg_camera01;
    sensor_msgs::CameraInfo ros_cameraInfoMsg_camera02;
    sensor_msgs::CameraInfo ros_cameraInfoMsg_camera03;

    cv_bridge::CvImage cv_bridge_img;

    ros::Publisher map_pub           = node.advertise<pcl::PointCloud<pcl::PointXYZ> >  ("hdl64e",                  1, true);
    ros::Publisher gps_pub           = node.advertise<sensor_msgs::NavSatFix>           ("oxts/gps",                1, true);
    ros::Publisher gps_pub_initial   = node.advertise<sensor_msgs::NavSatFix>           ("oxts/gps_initial",        1, true);
    ros::Publisher imu_pub           = node.advertise<sensor_msgs::Imu>                 ("oxts/imu",                1, true);
    ros::Publisher disp_pub          = node.advertise<stereo_msgs::DisparityImage>      ("preprocessed_disparity",  1, true);
    ros::Publisher bird_pub          = node.advertise<sensor_msgs::Image>               ("birdView",                1, true);
    ros::Publisher truth_pub         = node.advertise<sensor_msgs::Image>               ("truth",                   1, true);


    sensor_msgs::NavSatFix  ros_msgGpsFix;
    sensor_msgs::NavSatFix  ros_msgGpsFixInitial;   // This message contains the first reading of the file
    bool                    firstGpsData = true;    // Flag to store the ros_msgGpsFixInitial message
    sensor_msgs::Imu        ros_msgImu;

    ros::Subscriber sub = node.subscribe("/kitti_player/synch", 1, synchCallback);    // refs #600

    if (vm.count("help"))
    {
        cout << desc << endl;

        cout << "kitti_player needs a directory tree like the following:" << endl;
        cout << "├── calib_cam_to_cam.txt      " << endl;
        cout << "└── 2011_09_26_drive_0001_sync" << endl;
        cout << "    ├── image_00              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    │   └ timestamps.txt      " << endl;
        cout << "    ├── image_01              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    │   └ timestamps.txt      " << endl;
        cout << "    ├── image_02              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    │   └ timestamps.txt      " << endl;
        cout << "    ├── image_03              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    │   └ timestamps.txt      " << endl;
        cout << "    ├── oxts                  " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    │   └ timestamps.txt      " << endl;
        cout << "    ├── velodyne_points       " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    │     └ timestamps.txt    " << endl;
        cout << "    ├── result                " << endl;
        cout << "    │   └── ****.jpg          " << endl;
        cout << "    └── tracklet_labels.xml   " << endl << endl;
        return 1;
    }

    if (!(options.all_data || options.color || options.gps || options.grayscale || options.imu || options.velodyne))
    {
        ROS_WARN_STREAM("Job finished without playing the dataset. No 'publishing' parameters provided");
        node.shutdown();
        return 1;
    }

    dir_root             = options.path;
    dir_image00          = options.path;
    dir_image01          = options.path;
    dir_image02          = options.path;
    dir_image03          = options.path;
    dir_image04          = options.path;
    dir_oxts             = options.path;
    dir_velodyne_points  = options.path;
    dir_image04          = options.path;
    dir_result           = options.path;

    (*(options.path.end() - 1) != '/' ? dir_root            = options.path + "/"                      : dir_root            = options.path);
    (*(options.path.end() - 1) != '/' ? dir_result          = options.path + "/result/"               : dir_result          = options.path + "result/");
    (*(options.path.end() - 1) != '/' ? dir_image00         = options.path + "/image_00/data/"        : dir_image00         = options.path + "image_00/data/");
    (*(options.path.end() - 1) != '/' ? dir_image01         = options.path + "/image_01/data/"        : dir_image01         = options.path + "image_01/data/");
    (*(options.path.end() - 1) != '/' ? dir_image02         = options.path + "/image_02/data/"        : dir_image02         = options.path + "image_02/data/");
    (*(options.path.end() - 1) != '/' ? dir_image03         = options.path + "/image_03/data/"        : dir_image03         = options.path + "image_03/data/");
    (*(options.path.end() - 1) != '/' ? dir_image04         = options.path + "/disparities/"          : dir_image04         = options.path + "disparities/");
    (*(options.path.end() - 1) != '/' ? dir_oxts            = options.path + "/oxts/data/"            : dir_oxts            = options.path + "oxts/data/");
    (*(options.path.end() - 1) != '/' ? dir_velodyne_points = options.path + "/velodyne_points/data/" : dir_velodyne_points = options.path + "velodyne_points/data/");

    (*(options.path.end() - 1) != '/' ? dir_timestamp_image00    = options.path + "/image_00/"            : dir_timestamp_image00   = options.path + "image_00/");
    (*(options.path.end() - 1) != '/' ? dir_timestamp_image01    = options.path + "/image_01/"            : dir_timestamp_image01   = options.path + "image_01/");
    (*(options.path.end() - 1) != '/' ? dir_timestamp_image02    = options.path + "/image_02/"            : dir_timestamp_image02   = options.path + "image_02/");
    (*(options.path.end() - 1) != '/' ? dir_timestamp_image03    = options.path + "/image_03/"            : dir_timestamp_image03   = options.path + "image_03/");
    (*(options.path.end() - 1) != '/' ? dir_timestamp_oxts       = options.path + "/oxts/"                : dir_timestamp_oxts      = options.path + "oxts/");
    (*(options.path.end() - 1) != '/' ? dir_timestamp_velodyne   = options.path + "/velodyne_points/"     : dir_timestamp_velodyne  = options.path + "velodyne_points/");
    // Check all the directories
    if (
        (options.all_data       && (   (opendir(dir_image00.c_str())            == NULL) ||
                                       (opendir(dir_image01.c_str())            == NULL) ||
                                       (opendir(dir_image02.c_str())            == NULL) ||
                                       (opendir(dir_image03.c_str())            == NULL) ||
                                       (opendir(dir_oxts.c_str())               == NULL) ||
                                       (opendir(dir_velodyne_points.c_str())    == NULL)))
        ||
        (options.color          && (   (opendir(dir_image02.c_str())            == NULL) ||
                                       (opendir(dir_image03.c_str())            == NULL)))
        ||
        (options.grayscale      && (   (opendir(dir_image00.c_str())            == NULL) ||
                                       (opendir(dir_image01.c_str())            == NULL)))
        ||
        (options.imu            && (   (opendir(dir_oxts.c_str())               == NULL)))
        ||
        (options.gps            && (   (opendir(dir_oxts.c_str())               == NULL)))
        ||
        (options.stereoDisp     && (   (opendir(dir_image04.c_str())            == NULL)))
        ||
        (options.velodyne       && (   (opendir(dir_velodyne_points.c_str())    == NULL)))
        ||
        (options.timestamps     && (   (opendir(dir_timestamp_image00.c_str())      == NULL) ||
                                       (opendir(dir_timestamp_image01.c_str())      == NULL) ||
                                       (opendir(dir_timestamp_image02.c_str())      == NULL) ||
                                       (opendir(dir_timestamp_image03.c_str())      == NULL) ||
                                       (opendir(dir_timestamp_oxts.c_str())         == NULL) ||
                                       (opendir(dir_timestamp_velodyne.c_str())     == NULL)))

    )
    {
        ROS_ERROR("Incorrect tree directory , use --help for details");
        node.shutdown();
        return -1;
    }
    else
    {
        ROS_INFO_STREAM ("Checking directories...");
        ROS_INFO_STREAM (options.path << "\t[OK]");
    }

    //count elements in the folder

    if (options.all_data)
    {
        dir = opendir(dir_image02.c_str());
        while ((ent = readdir(dir)))
        {
            //skip . & ..
            len = strlen (ent->d_name);
            //skip . & ..
            if (len > 2)
                total_entries++;
        }
        closedir (dir);
    }
    else
    {
        bool done = false;
        if (!done && options.color)
        {
            total_entries = 0;
            dir = opendir(dir_image02.c_str());
            while ((ent = readdir(dir)))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len > 2)
                    total_entries++;
            }
            closedir (dir);
            done = true;
        }
        if (!done && options.grayscale)
        {
            total_entries = 0;
            dir = opendir(dir_image00.c_str());
            while ((ent = readdir(dir)))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len > 2)
                    total_entries++;
            }
            closedir (dir);
            done = true;
        }
        if (!done && options.gps)
        {
            total_entries = 0;
            dir = opendir(dir_oxts.c_str());
            while ((ent = readdir(dir)))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len > 2)
                    total_entries++;
            }
            closedir (dir);
            done = true;
        }
        if (!done && options.imu)
        {
            total_entries = 0;
            dir = opendir(dir_oxts.c_str());
            while ((ent = readdir(dir)))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len > 2)
                    total_entries++;
            }
            closedir (dir);
            done = true;
        }
        if (!done && options.velodyne)
        {
            total_entries = 0;
            dir = opendir(dir_oxts.c_str());
            while ((ent = readdir(dir)))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len > 2)
                    total_entries++;
            }
            closedir (dir);
            done = true;
        }
        if (!done && options.stereoDisp)
        {
            total_entries = 0;
            dir = opendir(dir_image04.c_str());
            while ((ent = readdir(dir)))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len > 2)
                    total_entries++;
            }
            closedir (dir);
            done = true;
        }
    }

    // Check options.startFrame and total_entries
    if (options.startFrame > total_entries)
    {
        ROS_ERROR("Error, start number > total entries in the dataset");
        node.shutdown();
        return -1;
    }
    else
    {
        entries_played = options.startFrame;
        ROS_INFO_STREAM("The entry point (frame number) is: " << entries_played);
    }

    if (options.viewer)
    {
        ROS_INFO_STREAM("Opening CV viewer(s)");
        if (options.color || options.all_data)
        {
            ROS_DEBUG_STREAM("color||all " << options.color << " " << options.all_data);
            cv::namedWindow("CameraSimulator Color Viewer", CV_WINDOW_AUTOSIZE);
            full_filename_image02 = dir_image02 + boost::str(boost::format("%010d") % 0 ) + ".png";
            cv_image02 = cv::imread(full_filename_image02, CV_LOAD_IMAGE_UNCHANGED);
            cv::waitKey(5);
        }
        if (options.grayscale || options.all_data)
        {
            ROS_DEBUG_STREAM("grayscale||all " << options.grayscale << " " << options.all_data);
            cv::namedWindow("CameraSimulator Grayscale Viewer", CV_WINDOW_AUTOSIZE);
            full_filename_image00 = dir_image00 + boost::str(boost::format("%010d") % 0 ) + ".png";
            cv_image00 = cv::imread(full_filename_image00, CV_LOAD_IMAGE_UNCHANGED);
            cv::waitKey(5);
        }
        if (options.viewDisparities || options.all_data)
        {
            ROS_DEBUG_STREAM("viewDisparities||all " << options.grayscale << " " << options.all_data);
            cv::namedWindow("Precomputed Disparities", CV_WINDOW_AUTOSIZE);
            full_filename_Disparities = dir_Disparities + boost::str(boost::format("%010d") % 0 ) + ".png";
            cv_disparities = cv::imread(full_filename_Disparities, CV_LOAD_IMAGE_UNCHANGED);
            cv::waitKey(5);
        }
        ROS_INFO_STREAM("Opening CV viewer(s)... OK");
    }

    // CAMERA INFO SECTION: read one for all

    ros_cameraInfoMsg_camera00.header.stamp = ros::Time::now();
    ros_cameraInfoMsg_camera00.header.frame_id = ros::this_node::getName();
    ros_cameraInfoMsg_camera00.height = 0;
    ros_cameraInfoMsg_camera00.width  = 0;
    //ros_cameraInfoMsg_camera00.D.resize(5);
    //ros_cameraInfoMsg_camera00.distortion_model=sensor_msgs::distortion_models::PLUMB_BOB;

    ros_cameraInfoMsg_camera01.header.stamp = ros::Time::now();
    ros_cameraInfoMsg_camera01.header.frame_id = ros::this_node::getName();
    ros_cameraInfoMsg_camera01.height = 0;
    ros_cameraInfoMsg_camera01.width  = 0;
    //ros_cameraInfoMsg_camera01.D.resize(5);
    //ros_cameraInfoMsg_camera00.distortion_model=sensor_msgs::distortion_models::PLUMB_BOB;

    ros_cameraInfoMsg_camera02.header.stamp = ros::Time::now();
    ros_cameraInfoMsg_camera02.header.frame_id = ros::this_node::getName();
    ros_cameraInfoMsg_camera02.height = 0;
    ros_cameraInfoMsg_camera02.width  = 0;
    //ros_cameraInfoMsg_camera02.D.resize(5);
    //ros_cameraInfoMsg_camera02.distortion_model=sensor_msgs::distortion_models::PLUMB_BOB;

    ros_cameraInfoMsg_camera03.header.stamp = ros::Time::now();
    ros_cameraInfoMsg_camera03.header.frame_id = ros::this_node::getName();
    ros_cameraInfoMsg_camera03.height = 0;
    ros_cameraInfoMsg_camera03.width  = 0;
    //ros_cameraInfoMsg_camera03.D.resize(5);
    //ros_cameraInfoMsg_camera03.distortion_model=sensor_msgs::distortion_models::PLUMB_BOB;

    if (options.color || options.all_data)
    {
        if (
            !(getCalibration(dir_root, "02", ros_cameraInfoMsg_camera02.K.data(), ros_cameraInfoMsg_camera02.D, ros_cameraInfoMsg_camera02.R.data(), ros_cameraInfoMsg_camera02.P.data()) &&
              getCalibration(dir_root, "03", ros_cameraInfoMsg_camera03.K.data(), ros_cameraInfoMsg_camera03.D, ros_cameraInfoMsg_camera03.R.data(), ros_cameraInfoMsg_camera03.P.data()))
        )
        {
            ROS_ERROR_STREAM("Error reading CAMERA02/CAMERA03 calibration");
            node.shutdown();
            return -1;
        }
        //Assume same height/width for the camera pair
        full_filename_image02 = dir_image02 + boost::str(boost::format("%010d") % 0 ) + ".png";
        cv_image02 = cv::imread(full_filename_image02, CV_LOAD_IMAGE_UNCHANGED);
        cv::waitKey(5);
        ros_cameraInfoMsg_camera03.height = ros_cameraInfoMsg_camera02.height = cv_image02.rows;// -1;TODO: CHECK, qui potrebbe essere -1
        ros_cameraInfoMsg_camera03.width  = ros_cameraInfoMsg_camera02.width  = cv_image02.cols;// -1;
    }

    if (options.grayscale || options.all_data)
    {
        if (
            !(getCalibration(dir_root, "00", ros_cameraInfoMsg_camera00.K.data(), ros_cameraInfoMsg_camera00.D, ros_cameraInfoMsg_camera00.R.data(), ros_cameraInfoMsg_camera00.P.data()) &&
              getCalibration(dir_root, "01", ros_cameraInfoMsg_camera01.K.data(), ros_cameraInfoMsg_camera01.D, ros_cameraInfoMsg_camera01.R.data(), ros_cameraInfoMsg_camera01.P.data()))
        )
        {
            ROS_ERROR_STREAM("Error reading CAMERA00/CAMERA01 calibration");
            node.shutdown();
            return -1;
        }
        //Assume same height/width for the camera pair
        full_filename_image00 = dir_image00 + boost::str(boost::format("%010d") % 0 ) + ".png";
        cv_image00 = cv::imread(full_filename_image00, CV_LOAD_IMAGE_UNCHANGED);
        cv::waitKey(5);
        ros_cameraInfoMsg_camera01.height = ros_cameraInfoMsg_camera00.height = cv_image00.rows;// -1; TODO: CHECK -1?
        ros_cameraInfoMsg_camera01.width  = ros_cameraInfoMsg_camera00.width  = cv_image00.cols;// -1;
    }

    boost::progress_display progress(total_entries) ;
    double cv_min, cv_max = 0.0f;
    ros::Publisher publisher_GT_RTK;
    publisher_GT_RTK = node.advertise<visualization_msgs::MarkerArray> ("/kitti_player/GT_RTK", 1);

    // This is the main KITTI_PLAYER Loop
    do
    {
        // this refs #600 synchMode
        if (options.synchMode)
        {
            if (waitSynch == true)
            {
                ROS_DEBUG_STREAM("Waiting for synch...");
                ros::spinOnce();
                continue;
            }
            else
            {
                ROS_DEBUG_STREAM("Run after received synch...");
                waitSynch = true;
            }
        }

        // single timestamp for all published stuff
        ros::Time current_timestamp = ros::Time::now();

        if (options.stereoDisp)
        {
            // Allocate new disparity image message
            stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage>();

            full_filename_image04 = dir_image04 + boost::str(boost::format("%010d") % entries_played ) + ".png";
            cv_image04 = cv::imread(full_filename_image04, CV_LOAD_IMAGE_GRAYSCALE);

            cv::minMaxLoc(cv_image04, &cv_min, &cv_max);

            disp_msg->min_disparity = (int)cv_min;
            disp_msg->max_disparity = (int)cv_max;

            disp_msg->valid_window.x_offset = 0;  // should be safe, checked!
            disp_msg->valid_window.y_offset = 0;  // should be safe, checked!
            disp_msg->valid_window.width    = 0;  // should be safe, checked!
            disp_msg->valid_window.height   = 0;  // should be safe, checked!
            disp_msg->T                     = 0;  // should be safe, checked!
            disp_msg->f                     = 0;  // should be safe, checked!
            disp_msg->delta_d               = 0;  // should be safe, checked!
            disp_msg->header.stamp          = current_timestamp;
            disp_msg->header.frame_id       = ros::this_node::getName();
            disp_msg->header.seq            = progress.count();

            sensor_msgs::Image& dimage = disp_msg->image;
            dimage.width  = cv_image04.size().width ;
            dimage.height = cv_image04.size().height ;
            dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            dimage.step = dimage.width * sizeof(float);
            dimage.data.resize(dimage.step * dimage.height);
            cv::Mat_<float> dmat(dimage.height, dimage.width, reinterpret_cast<float*>(&dimage.data[0]), dimage.step);

            cv_image04.convertTo(dmat, dmat.type());

            disp_pub.publish(disp_msg);

        }

        if (options.viewDisparities)
        {
            full_filename_Disparities = dir_Disparities + boost::str(boost::format("%010d") % entries_played ) + ".png";
            cv_disparities = cv::imread(full_filename_Disparities, CV_LOAD_IMAGE_UNCHANGED);
            cv::putText(cv_disparities, "KittiPlayer", cvPoint(20, 15), CV_FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 255, 0), 1, CV_AA);
            cv::putText(cv_disparities, boost::str(boost::format("%5d") % entries_played ), cvPoint(cv_disparities.size().width - 100, 30), CV_FONT_HERSHEY_DUPLEX, 1.0, cvScalar(0, 0, 255), 1, CV_AA);
            cv::waitKey(5);
        }

        if (options.color || options.all_data)
        {
            full_filename_image02 = dir_image02 + boost::str(boost::format("%010d") % entries_played ) + ".png";
            full_filename_image03 = dir_image03 + boost::str(boost::format("%010d") % entries_played ) + ".png";
            ROS_DEBUG_STREAM ( full_filename_image02 << endl << full_filename_image03 << endl << endl);

            cv_image02 = cv::imread(full_filename_image02, CV_LOAD_IMAGE_UNCHANGED);
            cv_image03 = cv::imread(full_filename_image03, CV_LOAD_IMAGE_UNCHANGED);

            if ( (cv_image02.data == NULL) || (cv_image03.data == NULL) )
            {
                ROS_ERROR_STREAM("Error reading color images (02 & 03)");
                ROS_ERROR_STREAM(full_filename_image02 << endl << full_filename_image03);
                node.shutdown();
                return -1;
            }

            if (options.viewer)
            {
                //display the left image only
                cv::imshow("CameraSimulator Color Viewer", cv_image02);
                //give some time to draw images
                cv::waitKey(5);
            }

            cv_bridge_img.encoding = sensor_msgs::image_encodings::BGR8;
            cv_bridge_img.header.frame_id = ros::this_node::getName();

            if (!options.timestamps)
            {
                cv_bridge_img.header.stamp = current_timestamp ;
                ros_msg02.header.stamp = ros_cameraInfoMsg_camera02.header.stamp = cv_bridge_img.header.stamp;
            }
            else
            {

                str_support = dir_timestamp_image02 + "timestamps.txt";
                ifstream timestamps(str_support.c_str());
                if (!timestamps.is_open())
                {
                    ROS_ERROR_STREAM("Fail to open " << timestamps);
                    node.shutdown();
                    return -1;
                }
                timestamps.seekg(30 * entries_played);
                getline(timestamps, str_support);
                cv_bridge_img.header.stamp = parseTime(str_support).stamp;
                ros_msg02.header.stamp = ros_cameraInfoMsg_camera02.header.stamp = cv_bridge_img.header.stamp;
            }
            cv_bridge_img.image = cv_image02;
            cv_bridge_img.toImageMsg(ros_msg02);

            if (!options.timestamps)
            {
                cv_bridge_img.header.stamp = current_timestamp;
                ros_msg03.header.stamp = ros_cameraInfoMsg_camera03.header.stamp = cv_bridge_img.header.stamp;
            }
            else
            {

                str_support = dir_timestamp_image03 + "timestamps.txt";
                ifstream timestamps(str_support.c_str());
                if (!timestamps.is_open())
                {
                    ROS_ERROR_STREAM("Fail to open " << timestamps);
                    node.shutdown();
                    return -1;
                }
                timestamps.seekg(30 * entries_played);
                getline(timestamps, str_support);
                cv_bridge_img.header.stamp = parseTime(str_support).stamp;
                ros_msg03.header.stamp = ros_cameraInfoMsg_camera03.header.stamp = cv_bridge_img.header.stamp;
            }

            cv_bridge_img.image = cv_image03;
            cv_bridge_img.toImageMsg(ros_msg03);

            pub02.publish(ros_msg02, ros_cameraInfoMsg_camera02);
            pub03.publish(ros_msg03, ros_cameraInfoMsg_camera03);

        }

        if (options.grayscale || options.all_data)
        {
            full_filename_image00 = dir_image00 + boost::str(boost::format("%010d") % entries_played ) + ".png";
            full_filename_image01 = dir_image01 + boost::str(boost::format("%010d") % entries_played ) + ".png";
            ROS_DEBUG_STREAM ( full_filename_image00 << endl << full_filename_image01 << endl << endl);

            cv_image00 = cv::imread(full_filename_image00, CV_LOAD_IMAGE_UNCHANGED);
            cv_image01 = cv::imread(full_filename_image01, CV_LOAD_IMAGE_UNCHANGED);

            if ( (cv_image00.data == NULL) || (cv_image01.data == NULL) )
            {
                ROS_ERROR_STREAM("Error reading color images (00 & 01)");
                ROS_ERROR_STREAM(full_filename_image00 << endl << full_filename_image01);
                node.shutdown();
                return -1;
            }

            if (options.viewer)
            {
                //display the left image only
                cv::imshow("CameraSimulator Grayscale Viewer", cv_image00);
                //give some time to draw images
                cv::waitKey(5);
            }

            cv_bridge_img.encoding = sensor_msgs::image_encodings::MONO8;
            cv_bridge_img.header.frame_id = ros::this_node::getName();

            if (!options.timestamps)
            {
                cv_bridge_img.header.stamp = current_timestamp;
                ros_msg00.header.stamp = ros_cameraInfoMsg_camera00.header.stamp = cv_bridge_img.header.stamp;
            }
            else
            {

                str_support = dir_timestamp_image02 + "timestamps.txt";
                ifstream timestamps(str_support.c_str());
                if (!timestamps.is_open())
                {
                    ROS_ERROR_STREAM("Fail to open " << timestamps);
                    node.shutdown();
                    return -1;
                }
                timestamps.seekg(30 * entries_played);
                getline(timestamps, str_support);
                cv_bridge_img.header.stamp = parseTime(str_support).stamp;
                ros_msg00.header.stamp = ros_cameraInfoMsg_camera00.header.stamp = cv_bridge_img.header.stamp;
            }
            cv_bridge_img.image = cv_image00;
            cv_bridge_img.toImageMsg(ros_msg00);

            if (!options.timestamps)
            {
                cv_bridge_img.header.stamp = current_timestamp;
                ros_msg01.header.stamp = ros_cameraInfoMsg_camera01.header.stamp = cv_bridge_img.header.stamp;
            }
            else
            {

                str_support = dir_timestamp_image02 + "timestamps.txt";
                ifstream timestamps(str_support.c_str());
                if (!timestamps.is_open())
                {
                    ROS_ERROR_STREAM("Fail to open " << timestamps);
                    node.shutdown();
                    return -1;
                }
                timestamps.seekg(30 * entries_played);
                getline(timestamps, str_support);
                cv_bridge_img.header.stamp = parseTime(str_support).stamp;
                ros_msg01.header.stamp = ros_cameraInfoMsg_camera01.header.stamp = cv_bridge_img.header.stamp;
            }
            cv_bridge_img.image = cv_image01;
            cv_bridge_img.toImageMsg(ros_msg01);

            pub00.publish(ros_msg00, ros_cameraInfoMsg_camera00);
            pub01.publish(ros_msg01, ros_cameraInfoMsg_camera01);

        }

        //Todo add tracklets_labels here
        if (options.velodyne || options.all_data)
        {
            header_support.stamp = current_timestamp;
            full_filename_velodyne = dir_velodyne_points + boost::str(boost::format("%010d") % entries_played ) + ".bin";
            full_filename_result   = dir_result + boost::str(boost::format("%010d") % entries_played );

            if (!options.timestamps)
                publish_velodyne(map_pub, bird_pub, truth_pub, full_filename_velodyne, options.object, entries_played, dir_root, full_filename_result, &header_support);
            else
            {
                str_support = dir_timestamp_velodyne + "timestamps.txt";
                ifstream timestamps(str_support.c_str());
                if (!timestamps.is_open())
                {
                    ROS_ERROR_STREAM("Fail to open " << timestamps);
                    node.shutdown();
                    return -1;
                }
                timestamps.seekg(30 * entries_played);
                getline(timestamps, str_support);
                header_support.stamp = parseTime(str_support).stamp;
                publish_velodyne(map_pub, bird_pub, truth_pub, full_filename_velodyne, options.object, entries_played, dir_root, full_filename_result, &header_support);
            }

        }

        if (options.gps || options.all_data)
        {
            header_support.stamp = current_timestamp; //ros::Time::now();
            if (options.timestamps)
            {
                str_support = dir_timestamp_oxts + "timestamps.txt";
                ifstream timestamps(str_support.c_str());
                if (!timestamps.is_open())
                {
                    ROS_ERROR_STREAM("Fail to open " << timestamps);
                    node.shutdown();
                    return -1;
                }
                timestamps.seekg(30 * entries_played);
                getline(timestamps, str_support);
                header_support.stamp = parseTime(str_support).stamp;
            }

            full_filename_oxts = dir_oxts + boost::str(boost::format("%010d") % entries_played ) + ".txt";
            if (!getGPS(full_filename_oxts, &ros_msgGpsFix, &header_support))
            {
                ROS_ERROR_STREAM("Fail to open " << full_filename_oxts);
                node.shutdown();
                return -1;
            }

            if (firstGpsData)
            {
                // this refs to BUG #551 - If a starting frame is specified, a wrong
                // initial-gps-fix is taken. Fixing this issue forcing filename to
                // 0000000001.txt
                // The FULL dataset should be always downloaded.
                full_filename_oxts = dir_oxts + "0000000001.txt";
                if (!getGPS(full_filename_oxts, &ros_msgGpsFix, &header_support))
                {
                    ROS_ERROR_STREAM("Fail to open " << full_filename_oxts);
                    node.shutdown();
                    return -1;
                }
                ROS_DEBUG_STREAM("Setting initial GPS fix at " << endl << ros_msgGpsFix);
                firstGpsData = false;
                ros_msgGpsFixInitial = ros_msgGpsFix;
                ros_msgGpsFixInitial.header.frame_id = "/local_map";
                ros_msgGpsFixInitial.altitude = 0.0f;
            }

            gps_pub.publish(ros_msgGpsFix);
            gps_pub_initial.publish(ros_msgGpsFixInitial);

            // this refs #522 - adding GPS-RTK Markers (published RVIZ markers)
            if (options.gpsReferenceFrame.length() > 1)
            {

                Xy xyFromLatLon;
                xyFromLatLon = latlon2xy_helper(ros_msgGpsFix.latitude, ros_msgGpsFix.longitude);

                static visualization_msgs::MarkerArray marker_array_GT_RTK;
                visualization_msgs::Marker RTK_MARKER;

                static int gps_track = 1;
                RTK_MARKER.header.frame_id = options.gpsReferenceFrame;
                RTK_MARKER.header.stamp = current_timestamp;
                RTK_MARKER.ns = "RTK_MARKER";
                RTK_MARKER.id = gps_track++; //unused
                RTK_MARKER.type = visualization_msgs::Marker::CYLINDER;
                RTK_MARKER.action = visualization_msgs::Marker::ADD;
                RTK_MARKER.pose.orientation.w = 1;
                RTK_MARKER.scale.x = 0.5;
                RTK_MARKER.scale.y = 0.5;
                RTK_MARKER.scale.z = 3.5;
                RTK_MARKER.color.a = 0.80;
                RTK_MARKER.color.r = 0;
                RTK_MARKER.color.g = 0.0;
                RTK_MARKER.color.b = 1.0;
                RTK_MARKER.pose.position.x = xyFromLatLon.x;
                RTK_MARKER.pose.position.y = xyFromLatLon.y;
                RTK_MARKER.pose.position.z = 0;

                ROS_DEBUG_STREAM(RTK_MARKER.pose.position.x << "\t" << RTK_MARKER.pose.position.y);

                marker_array_GT_RTK.markers.push_back(RTK_MARKER);

                // Push back line_list
                publisher_GT_RTK.publish(marker_array_GT_RTK);

            }
        }

        if (options.imu || options.all_data)
        {
            header_support.stamp = current_timestamp; //ros::Time::now();
            if (options.timestamps)
            {
                str_support = dir_timestamp_oxts + "timestamps.txt";
                ifstream timestamps(str_support.c_str());
                if (!timestamps.is_open())
                {
                    ROS_ERROR_STREAM("Fail to open " << timestamps);
                    node.shutdown();
                    return -1;
                }
                timestamps.seekg(30 * entries_played);
                getline(timestamps, str_support);
                header_support.stamp = parseTime(str_support).stamp;
            }


            full_filename_oxts = dir_oxts + boost::str(boost::format("%010d") % entries_played ) + ".txt";
            if (!getIMU(full_filename_oxts, &ros_msgImu, &header_support))
            {
                ROS_ERROR_STREAM("Fail to open " << full_filename_oxts);
                node.shutdown();
                return -1;
            }
            imu_pub.publish(ros_msgImu);

        }

        ++progress;
        entries_played++;

        if (!options.synchMode)
            loop_rate.sleep();
    }
    while (entries_played <= total_entries - 1 && ros::ok());


    if (options.viewer)
    {
        ROS_INFO_STREAM(" Closing CV viewer(s)");
        if (options.color || options.all_data)
            cv::destroyWindow("CameraSimulator Color Viewer");
        if (options.grayscale || options.all_data)
            cv::destroyWindow("CameraSimulator Grayscale Viewer");
        if (options.viewDisparities)
            cv::destroyWindow("Reprojection of Detected Lines");
        ROS_INFO_STREAM(" Closing CV viewer(s)... OK");
    }


    ROS_INFO_STREAM("Done!");
    node.shutdown();

    return 0;
}

