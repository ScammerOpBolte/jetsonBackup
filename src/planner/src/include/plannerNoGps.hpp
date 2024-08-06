#ifndef IRC_PLANNER_H_
#define IRC_PLANNER_H_
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <iterator>
#include <algorithm>
#include <map>
#include <sensor_msgs/NavSatFix.h>
#include <sensors/imu_data.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>
#include <planner/Data.h>
#include <planner/img.h>
#include <stereo/tag.h>
#include <stereo/yolotag.h>
#include <std_msgs/Int32.h>

namespace planner
{
    // Static Variable Definitions
    // Rover Dimensions and Physical Properties
    const double kRoverLength = 1.40;
    const double kRoverBreadth =  1.11;
    const double kLidarPos = 0.43;// y distance from the Top-Center of the Rover
    // const double kRoverLength = 0.5;
    // const double kRoverBreadth = 0.4;
    // const double kLidarPos = 0.02;

    // Rover Speed & Obstacle constraints and parameters

    const double kMaxLinearVel = 0.5;
    const double kMinLinearVel = 0;

    const double kMaxAngularVel = 1.12;
    const double kMinAngularVel = 0;

    const double kMaxObsThreshold = 1.2;
    const double kMinObsThreshold = 0.3;
    const double kMinYObsThreshold = 0.3;

    const double kMaxXObjThreshold = 1.0;
    const double kMinXObjThreshold = 0.5;
    const double kMaxYObjThreshold = 1.0;
    const double kMinYObjThreshold = 0.3;

    // Sensor Data Thresholds

    const int kLidarAngularRange = 360;
    const int kLidarSamples = 430 ;
    // Goal Distance Threshold
    const double kDistanceTHreshold = 1;
    // const int task_goal_count = 7;
    // Enum for Rover state
    // Class defines different states of the rover during the misson including:
    // 1. Search Pattern following
    // 2. Aruco Tag Following
    // 3. Coordinate Following
    // 4. Obstacle Avoidance
    // 5. Data Analysis

    enum State
    {
        kSearchPattern,
        kArucoFollowing,
        kCoordinateFollowing,
        kObstacleAvoidance,
        kDataAnalysis
    };

    enum SearchPatternType
    {
        kGoalCircle,
        kPatternFollowing,
        kTerminate,
    };

    // Struct for Latitude and Longitude
    typedef struct Coordinates
    {
        double latitude;
        double longitude;
    } Coordinates;

    class EquationGenerator
    {
    public:
        // Takes in two coordinate points & returns equation of a line
        // @returns m,c where y = mx + c

        std::vector<double> straightLineEquation(double x1, double y1, double x2, double y2);

        // Takes in two coordniates (vertex and focus) & return equation of parabola
        // @returns a,b,c where y = ax^2 + bx + c

        std::vector<double> parabolicEquation(double vx, double vy, double fx, double fy);
    };

    // TODO(Saatwik): calculations happening in search pattern object
    class SearchPattern
    {
    protected:
        // geometry_msgs::Twist hardTurn(double time);

        // geometry_msgs::Twist moveForward(double time);
    };

    class Controller : private SearchPattern
    {
    protected:
        void obstacleAvoidance();

        void objectFollowing();

        bool search_pattern_flag = false;

        // 0 => Start Search Pattern
        // 1 => Hard turn right
        // 2 => Hard turn left
        // 3 => Returning to a specified angle
        // 1 => Inverted circle Pattern
        // 2 => Hard Turn Right
        int search_pattern_function = 0;
    };

    class StateClassifier
    {
    protected:
        // Truthtable select lines

        bool gps_coord_reached = false;
        bool aruco_detect = false;
        bool obstacle_detect = false;
        bool goal_reached = false;
        float angularOffset=30;

        // Rover state variables
        State CurrState;
        State PrevState;
        SearchPatternType FollowPattern = kPatternFollowing;

        // Returns a enum of rover state implementing the turth table below:
        // +========+=======+============+=======+===================+
        // | GPS    | Aruco |  Obstacle  | Goal  | State             |
        // +========+=======+============+=======+===================+
        // | 1      | 0     | 0          | 0     | GPS Following     |
        // | 0      | 0     | 0          | 0     | Search Pattern    |
        // | 0/1    | 1     | 0          | 0     | Aruco Following   |
        // | 0/1    | 0/1   | 1          | 0     | Obstacle Avoid    |
        // | 0/1    | 0/1   | 0/1        | 1     | Data Analysis     |
        // +--------+-------+------------+-------+-------------------+

        void RoverStateClassifier();
        
    };

    class SensorInterpreter : private StateClassifier
    {
    public:
        // Constructor
        SensorInterpreter()
        {
            lidar_center = kLidarSamples / 2;
            lidar_min_range =  lidar_center - ((atan(kRoverBreadth / 2 / kLidarPos) * 180 / M_PI) * kLidarSamples / kLidarAngularRange);
            lidar_max_range = lidar_center + ((atan(kRoverBreadth / 2 / kLidarPos) * 180 / M_PI) * kLidarSamples / kLidarAngularRange);
                std::cout<<lidar_min_range<<' '<<lidar_max_range<<std::endl;
        }

    protected:
        // Sensor Data Variables
        ros::NodeHandle n;
        double aruco_x = 100, aruco_y = 100, obs_x = 100, obs_y = 100;
        double current_orientation, dest_orientation, coordinate_bearing;
        double curr_time, temp_time;
        Coordinates curr_location, dest_location;
        int goal_type;
        double dest_distance, bearing_angle;
        std::vector<float> lidar_samples;
        int lidar_center, lidar_min_range, lidar_max_range;
        int goal_count = 0;
        ros::ServiceClient client = n.serviceClient<planner::Data>("data_saver");
        ros::ServiceClient client2 = n.serviceClient<planner::img>("image_saver");
        ros::Publisher state_pub = n.advertise<std_msgs::Int32>("/state_topic", 1);

        // Output Variables

        std::vector<double> output_velocity;
        std::map<int, Coordinates> obj_locations;

        // Setters & Getters

        void setGPSStatus(bool status);
        void setArucoStatus(bool status);
        void setGoalStatus();
        State getState(bool current);
        bool getObstacleStatus();
        bool getGPSStatus();
        SearchPatternType getSearchPatternType();
        void setSearchPatternType(SearchPatternType state);

        void setAngularOffset(float offset);
        float getAngularOffset();

        // Call State Classifier function

        void callStateClassifier();

        // Returns the distance between two coordniates on the Earth

        double haversine(Coordinates curr, Coordinates dest);

        // Returns the bearing of the rover between two coordinates on the Earth

        void coordinateBearing();

        // Returns the least angle to travel between two IMU angles

        double bearing(double curr, double dest);

        // Sets the obstacle detected boolean of the Rover based on Lidar Data

        void obstacleClassifier();

        // Function for analysing arrow coordinates and actions after each sub-goal

        void dataAnalyzer();

        // Setting goal coordiantes for the task

        void getCoordinates();
    };
    class SensorCallback : private SensorInterpreter, private Controller
    {
    public:
        SensorCallback(
            std::string imu_topic,
            std::string aruco_topic,
            std::string gps_topic,
            std::string lidar_topic)
        {
            // Requesting GPS Coordinate input from the user

            getCoordinates();

            // setting custom callback queue for node

            nh.setCallbackQueue(&stack_run);

            // Initializing Publishers

            vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

            // Initializing Subscribers

            imu_sub = n.subscribe(imu_topic, 1, &SensorCallback::imuCallback, this);
            gps_sub = n.subscribe(gps_topic, 1, &SensorCallback::gpsCallback, this);
            lidar_sub = n.subscribe(lidar_topic, 1, &SensorCallback::lidarCallback, this);
            aruco_sub = n.subscribe(aruco_topic, 1, &SensorCallback::arucoCallback, this);

            // aruco_yolo_sub = n.subscribe(aruco_yolo_topic, 1, &SensorCallback::arucoYoloCallback, this);
            // stack_sub = nh.subscribe()

            //  Formation of Equations

            obs_avoid_linear = equation_generation.straightLineEquation(kMinObsThreshold, 0, kMaxObsThreshold, kMaxLinearVel);
            obs_avoid_angular = equation_generation.straightLineEquation(kRoverBreadth / 2, 0, 0, kMaxAngularVel);
            obj_follow_linear = equation_generation.straightLineEquation(2, kMaxLinearVel, 0.5, 0);
            obj_follow_angular = equation_generation.straightLineEquation(0, 0, 16, kMaxAngularVel);
            
        };

        void setAngOffset(float offset){
            setAngularOffset(offset);
        };

        float getAngOffset(){
            return getAngularOffset();
        };

    private:
        // ROS Variables

        ros::NodeHandle nh;
        ros::CallbackQueue stack_run;
        ros::Publisher vel_pub;
        ros::Subscriber imu_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber lidar_sub;
        ros::Subscriber aruco_sub;
        // ros::Subscriber aruco_yolo_sub;
        ros::Subscriber stack_sub;

        // Creating Equation Generation Class

        EquationGenerator equation_generation;

        // Equation Constants

        std::vector<double> obs_avoid_linear;
        std::vector<double> obs_avoid_angular;
        std::vector<double> obj_follow_linear;
        std::vector<double> obj_follow_angular;
        std::vector<double> bearing_equation;

        // Velocity publisher variable

        geometry_msgs::Twist velocity;

        // IMU Callback

        void imuCallback(const sensors::imu_data::ConstPtr &imu_msg);

        // GPS Callback

        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &fix_);

        // LiDar Callback

        void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &las);

        // Aruco Tag Detection Callback

        void arucoCallback(const stereo::yolotag::ConstPtr &aruco);

        // void arucoYoloCallback(const stereo::yolotag::ConstPtr &msg);

        // Individual thread for publishing velocity based on the sensor data

        void stackRun();

        void publishVel(geometry_msgs::Twist& msg);

        // Calling Search Pattern following function from controller class

        void callSearchPattern(SearchPatternType search_pattern_type);

        // Function to publish velocity for Obstacle Avoidance

        void obstacleAvoidance();

        // Function to publish velocity to Follow Objects

        void objectFollowing();

        // Function to publish velocity to follow GPS Coordinates

        void coordinateFollowing();

        // Set Search Pattern Flag

        void setSearchPatternFlag (bool a);

        // Get Searcg Pattern Flag
        
        bool getSearchPatternFlag ();
    };

    // Function Definitons for State Classifier Class

    void StateClassifier::RoverStateClassifier()
    {
        
        if (goal_reached == true)
        {
            if (CurrState != kDataAnalysis)
                PrevState = CurrState;
            CurrState = kDataAnalysis;
            FollowPattern = kGoalCircle;
        }
        else if (obstacle_detect == true)
        {
            if (CurrState != kObstacleAvoidance)
                PrevState = CurrState;
            CurrState = kObstacleAvoidance;
        }
        // else if (gps_coord_reached == false)
        // {
        //     if (CurrState != kCoordinateFollowing)
        //         PrevState = CurrState;
        //     CurrState = kCoordinateFollowing;
        // }
        else if (aruco_detect == true)
        {
            if (CurrState != kArucoFollowing)
                PrevState = CurrState;
            CurrState = kArucoFollowing;
        }
        else
        {
            if (CurrState != kSearchPattern)
                PrevState = CurrState;
            // angularOffset=15;            
            CurrState = kSearchPattern;
            if (FollowPattern != kTerminate)
                FollowPattern = kPatternFollowing;
        }
    };


    // Function Definitions for Sensor Interpreter Class

    void SensorInterpreter::setGPSStatus(bool status)
    {
        gps_coord_reached = status;
    };

    void SensorInterpreter::setArucoStatus(bool status)
    {
        aruco_detect = status;
    };

    void SensorInterpreter::setAngularOffset(float offset){
        angularOffset=offset;
    };

    float SensorInterpreter::getAngularOffset(){
        return angularOffset;
    };

    // void SensorInterpreter::setFollowPattern(State);

    void SensorInterpreter::setGoalStatus()
    {
        std_msgs::Int32 msg;
        if (goal_type==1)
        {
            if (sqrt(pow(aruco_x, 2) + pow(aruco_y, 2)) <= 2)
            {
                if (goal_reached == false){
                    std::cout << sqrt(pow(aruco_x, 2) + pow(aruco_y, 2)) << std::endl;
                    ROS_INFO_STREAM("ArUco Tag Reached");
                    // ROS_INFO_STREAM("GOAL HAS BEEN REACHED!");
                }
                goal_reached = true;
                msg.data=2;
                state_pub.publish(msg);
            }
            else
                goal_reached = false;
                msg.data=1;
                state_pub.publish(msg);
        }
        else if (goal_type==0){
            if (getGPSStatus())
            {
                if (goal_reached == false){
                    std::cout <<dest_distance<< std::endl;
                    ROS_INFO_STREAM("GPS Coordinate Reached");
                    // ROS_INFO_STREAM("GOAL HAS BEEN REACHED!");
                }
                goal_reached = true;
                msg.data=2;
                state_pub.publish(msg);
            }
            else
                goal_reached = false;
                msg.data=1;
                state_pub.publish(msg);
        }
    }

    State SensorInterpreter::getState(bool curr)
    {
        if (curr = true)
            return CurrState;
        else
            return PrevState;
    };

    bool SensorInterpreter::getObstacleStatus()
    {
        return obstacle_detect;
    };

    bool SensorInterpreter::getGPSStatus()
    {
        return gps_coord_reached;
    };

    SearchPatternType SensorInterpreter::getSearchPatternType()
    {
        return FollowPattern;
    };

    void SensorInterpreter::setSearchPatternType(SearchPatternType state)
    {
        FollowPattern == state;
    };

    void SensorInterpreter::callStateClassifier()
    {
        RoverStateClassifier();
    }

    double SensorInterpreter::haversine(Coordinates curr, Coordinates dest)
    {
        double distance;
        double dLat = (dest.latitude - curr.latitude);
        double dLon = (dest.longitude - curr.longitude);
        double h = sin(dLat * 0.5) * sin(dLat * 0.5) + sin(dLon * 0.5) * sin(dLon * 5) * cos(curr.latitude * cos(dest.latitude));
        distance = asin(sqrt(h)) * 6367 * 2 * 1000;
        return distance;
    };

    void SensorInterpreter::coordinateBearing()
    {
        
        double dLon = (curr_location.longitude - dest_location.longitude);
        double y = sin(dLon) * cos(curr_location.latitude);
        double x = cos(dest_location.latitude) * sin(curr_location.latitude) - sin(dest_location.latitude) * cos(curr_location.latitude) * cos(dLon);
        double heading = atan2(y, x);
        if (heading < 0)
            heading = 2 * M_PI + heading;

        // Calculating bearing in -Pi to Pi form

        double orientation_radian = current_orientation * M_PI / 180;

        coordinate_bearing = heading - orientation_radian;

        // std::cout<<"Heading: "<<heading<<std::endl;
        // std::cout<<"orientation_radian: "<<orientation_radian<<std::endl;
        // std::cout<<"coordinate_bearing 1 : "<<coordinate_bearing<<std::endl;
        if (coordinate_bearing > M_PI)
            coordinate_bearing = coordinate_bearing - 2 * M_PI;
        else if (coordinate_bearing < -M_PI)
            coordinate_bearing = coordinate_bearing + 2 * M_PI;
        // std::cout<<"coordinate_bearing 2 : "<<coordinate_bearing<<std::endl;


    };
    double SensorInterpreter::bearing(double dest, double curr)
    {
        // current orientation ranges from -180 to 180 and destination orientation ranges from -180 to 180
        if (dest - curr > 180)
            return -(360 - (dest - curr));
        else if (dest - curr > 0)
            return (dest - curr);
        else if (dest - curr < -180)
            return (360 - abs(dest - curr));
        else
            return -(dest - curr);
    };

    void SensorInterpreter::obstacleClassifier()
    {
        // Variable Definiton

        float min_value = kMaxObsThreshold+1;
        double temp_angle = 0;
        int temp_sample = 0;

        // Finding obstacle in the given range and calcualting its coordinates wrt Lidar
        
        for (int i = lidar_min_range; i <= lidar_max_range; i++)
        {
            if (lidar_samples[i] <= min_value && lidar_samples[i] > kMinObsThreshold)
            {
                temp_sample = i;
                min_value = lidar_samples[temp_sample];
            }
        }
        temp_angle = abs(lidar_center - temp_sample) * kLidarAngularRange / kLidarSamples;
        obs_x = min_value * cos(temp_angle * M_PI / 180);
        obs_y = min_value * sin(temp_angle * M_PI / 180);
        if(lidar_center < temp_sample)
            obs_y = -obs_y;

        std::cout<<"Sample"<<temp_sample<<"obs_x"<<obs_x<<"obs_y"<<obs_y<<std::endl;
        std::cout<<"State: "<<getState(true)<<std::endl;
        // Classifying if obstacle is in path

        // If aruco is detected as obstacle

        if (aruco_detect == true && abs(obs_x - aruco_x) < 0.3 && abs(obs_y - aruco_y) < 0.3)
        {
            obstacle_detect = false;
            return;
        }

        // If obstacle is in the path of the rover

        if (min_value < kMaxObsThreshold && obs_x > kMinObsThreshold && obs_x < kRoverBreadth + 0.09 / 2)
            obstacle_detect = true;
        else
            obstacle_detect = false;
    };

    void SensorInterpreter::dataAnalyzer()
    {   
        if (PrevState == kArucoFollowing && goal_count == 7)
        {

            ROS_INFO_STREAM("Aruco Reached!");
            obj_locations.insert(std::pair<int, Coordinates>(obj_locations.size() + 1, curr_location));
            ROS_INFO_STREAM("Saving coordinates to CSV...");
            
            planner::Data srv;
            srv.request.status=true;
            if (client.call(srv)){         
                ROS_INFO("Success: %d", srv.response.success);
            }
            else{
                ROS_ERROR("Failed to call service data_saver");
            }

            planner::img image;
            image.request.status=true;
            if (client2.call(image)){         
                ROS_INFO("Success: %d", image.response.success);
            }
            else{
                ROS_ERROR("Failed to call service img_saver");
            }

            ROS_INFO_STREAM("Task complete! Aruco coordinates have been saved CSV.");
            FollowPattern = kTerminate;
        }

        else if (PrevState == kArucoFollowing)
        {
            ROS_INFO_STREAM("Aruco Reached!");
            ROS_INFO_STREAM("Checking for similar Aruco coordinates...");

            std::map<int, Coordinates>::iterator it = obj_locations.begin();
            bool same_coord = false;

            while (it != obj_locations.end())
            {
                Coordinates temp = it->second;
                double distance =  haversine(temp, curr_location);
                if (distance < 3)
                {
                    same_coord = true;
                    ROS_INFO_STREAM("Duplicate Aruco detected going back to GPS Coordinate...");
                    dest_distance =  haversine(curr_location, dest_location);
                    if (getGPSStatus() == false && dest_distance > 2)
                        setGPSStatus(false);
                    else if (getGPSStatus() == false && dest_distance < 2)
                        setGPSStatus(true);
                    return;
                }
            }
            if (same_coord == false)
            {
                goal_count++;
                ROS_INFO_STREAM("Unique Aruco found! Saving to map and updating CSV");
                obj_locations.insert(std::pair<int, Coordinates>(obj_locations.size() + 1, curr_location));

                planner::Data srv;
                srv.request.status=true;
                if (client.call(srv)){         
                    ROS_INFO("Success: %d", srv.response.success);
                }
                else{
                    ROS_ERROR("Failed to call service data_saver");
                }

                planner::img image;
                image.request.status=true;
                if (client2.call(image)){         
                    ROS_INFO("Success: %d", image.response.success);
                }
                else{
                    ROS_ERROR("Failed to call service img_saver");
                }
                
                ROS_INFO_STREAM("Flashing green light...");
                
                // TODO(Saatwik): setup green light flashing publish message

                ros::Duration(5).sleep();
                // setGPSStatus(false);
                getCoordinates();
            }
        }

        else if (PrevState == kCoordinateFollowing)
        {
            ROS_INFO_STREAM("Coordinate Reached!");      
            goal_count++;
   
            ROS_INFO_STREAM("Flashing green light...");
                
            // TODO(Saatwik): setup green light flashing publish message

            ros::Duration(5).sleep();
            getCoordinates();
            
        }
    };

    // Setting goal coordiantes for the task

    void SensorInterpreter::getCoordinates()
    {
        ROS_INFO_STREAM("Enter Latitude and Longitude of the goal:");
        std::cin >> dest_location.latitude >> dest_location.longitude;
        ROS_INFO_STREAM("Enter type of the goal (0 for gps 1 for object): ");
        std::cin >> goal_type;
        // if (dest_location.latitude == 0 && dest_location.longitude == 0)
        //     ROS_INFO_STREAM("Task Completed.");
        dest_location.latitude = dest_location.latitude * M_PI / 180;
        dest_location.longitude = dest_location.longitude * M_PI / 180;
        setGPSStatus(false);
        // setSearchPatternType(kTerminate);
    };

    // Function Definitions for Sensor Callback Class

    void SensorCallback::imuCallback(const sensors::imu_data::ConstPtr &imu_msg)
    {
        current_orientation = imu_msg->orientation.z;
    };

    void SensorCallback::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &fix_)
    {
        curr_location.latitude = fix_->latitude * M_PI / 180;
        curr_location.longitude = fix_->longitude * M_PI / 180;

        // Calulating distance using Haversine formula

        dest_distance =  haversine(curr_location, dest_location);
        // ROS_INFO_STREAM(getGPSStatus());
        // ROS_INFO_STREAM(dest_distance);

        // Calculating heading to travel

        coordinateBearing();

        if (getGPSStatus() == false && dest_distance > 2)
            setGPSStatus(false);

        else if (getGPSStatus() == false && dest_distance < 2)
            setGPSStatus(true);
        
        // else if (getGPSStatus() == true && dest_distance > 2)
        //     setGPSStatus(false);

        else if(goal_type==1 && getGPSStatus() == true && dest_distance > 25)
        {
            ROS_INFO_STREAM("Search Pattern did not find Aruco Markers");
            ROS_INFO_STREAM("Returning to GPS Coordinate...");
            setGPSStatus(false);
        }
    };

    void SensorCallback::lidarCallback(const sensor_msgs::LaserScan::ConstPtr &las)
    {
        lidar_samples = las->ranges;
        obstacleClassifier();
        if (getObstacleStatus() == true)
            search_pattern_function = 0;
        stackRun();
    };

    void SensorCallback::arucoCallback(const stereo::yolotag::ConstPtr &aruco)
    {
        // ROS_INFO("%d",aruco->isFound);
        if (aruco->isFound == true)
        {
            search_pattern_function = 0;
            aruco_x = aruco->x;
            aruco_y = aruco->y;
            setArucoStatus(true);
        }
        else
        {
            setArucoStatus(false);
            aruco_x = 100;
            aruco_y = 100;
        }
    };

    // void SensorCallback::arucoYoloCallback(const stereo::yolotag::ConstPtr &msg)
    // {
    //     ROS_INFO("%d",aruco->isFound);
    //     if (msg->isFound == true)
    //     {
    //         search_pattern_function = 0;
    //         aruco_x = msg->x;
    //         aruco_y = msg->y;
    //         setArucoStatus(true);
    //     }
    //     else
    //     {
    //         setArucoStatus(false);
    //         aruco_x = 100;
    //         aruco_y = 100;
    //     }
    // };

    void SensorCallback::stackRun()
    {
        setGoalStatus();

        callStateClassifier();
        
        // std::cout<<"State: "<<getState(true)<<std::endl;

        if (search_pattern_flag == false)
        {
            
            switch (getState(true))
            {

                // Function to Analyze data when rover reaches Aruco Tag

            case kDataAnalysis:
                velocity.linear.x = 0;
                velocity.angular.z = 0;
                publishVel(velocity);
                dataAnalyzer();
                if(goal_type!=0)
                    callSearchPattern(getSearchPatternType());
                break;

                // Function to publish velocity when in Aurco Tag Following

            case kArucoFollowing:
                objectFollowing();
                break;

                // Function to publish velocity when in Obstacle Avoidance

            case kObstacleAvoidance:
                obstacleAvoidance();
                break;

                // Function to publish velocity when following GPS Coordinate

            case kCoordinateFollowing:
                coordinateFollowing();
                break;

                // Function to Follow Search Pattern

            case kSearchPattern:
                callSearchPattern(getSearchPatternType());
                break;
            }
        }

        else if (getState(true) == kCoordinateFollowing)
            coordinateFollowing();
        else
            callSearchPattern(getSearchPatternType());
    };

    // Function Definitons for Equation Generator Class
    void SensorCallback::publishVel(geometry_msgs::Twist& msg)
    {
        if (msg.linear.x<0)
        {
            msg.linear.x=0;
        }
        vel_pub.publish(msg);
    }
    
    std::vector<double> EquationGenerator::straightLineEquation(double x1, double y1, double x2, double y2)
    {
        std::vector<double> temp;

        double m = (y2 - y1) / (x2 - x1);
        double c = -((y2 - y2) / (x2 - x1) * x1) + y1;

        temp.push_back(m);
        temp.push_back(c);

        return temp;
    };

    std::vector<double> EquationGenerator::parabolicEquation(double h, double k, double fx, double f)
    {
        std::vector<double> temp;

        double a = 1 / (4 * (f - k));
        double b = (-2 * h) / (4 * (f - k));
        double c = ((h * h) / (4 * (f - k))) + k;

        temp.push_back(a);
        temp.push_back(b);
        temp.push_back(c);

        return temp;
    };

    // Function Definitons for Search Pattern Class

    // geometry_msgs::Twist SearchPattern::hardTurn(double time){};

    // geometry_msgs::Twist SearchPattern::moveForward(double time){};

    // Function Definition for Controller Class

    void SensorCallback::callSearchPattern(SearchPatternType search_pattern_type)
    {
        switch (search_pattern_type)
        {

        case kGoalCircle:
            if (getSearchPatternFlag() == false)
            {
                ROS_INFO_STREAM("Turning 90 Degrees Right..");
                // curr_time = ros::Time::now().toSec();
                // temp_time = (ros::Time::now() + ros::Duration(100)).toSec();
                setSearchPatternFlag(true);
                dest_orientation = current_orientation + 90;
                if (dest_orientation > 360)
                    dest_orientation = 360 - dest_orientation;
                bearing_equation = equation_generation.straightLineEquation(90, kMaxAngularVel, 0, 0);
            }
            else
            {
                curr_time = ros::Time::now().toSec();
                if (abs(bearing(dest_orientation, current_orientation)) < 5)
                {
                    ROS_INFO_STREAM("Turning Complete...");
                    // ROS_INFO_STREAM("Moving in a Circle...");
                    setSearchPatternFlag(false);
                    // temp_time = (ros::Time::now() + ros::Duration(20)).toSec();
                    velocity.linear.x = 0;
                    velocity.angular.z = 0;
                    publishVel(velocity);
                }
                else if (abs(bearing(dest_orientation, current_orientation)) > 5)
                {
                    velocity.linear.x = 0;
                    velocity.angular.z = -(abs(bearing(dest_orientation, current_orientation)) * bearing_equation[0] + bearing_equation[1]);
                    publishVel(velocity);
                }
            }
            break;
        case kPatternFollowing:
            switch (search_pattern_function)
            {
            case 0:
                ROS_INFO_STREAM("Starting Search Pattern: Moving Forward...");
                curr_time = ros::Time::now().toSec();
                temp_time = (ros::Time::now() + ros::Duration(10)).toSec();
                search_pattern_function = 1;
            case 1:
                curr_time = ros::Time::now().toSec();
                if (temp_time >= curr_time)
                {
                    velocity.linear.x = kMaxLinearVel;
                    velocity.angular.z = 0;
                    publishVel(velocity);
                }
                else
                {
                    ROS_INFO_STREAM("Search Pattern: Hard Turning Right");
                    dest_orientation = current_orientation + 135;
                    if (dest_orientation > 360)
                        dest_orientation = dest_orientation - 360;
                    search_pattern_function = 2;
                    velocity.linear.x = 0;
                    velocity.angular.z = 0;
                    publishVel(velocity);
                    bearing_equation = equation_generation.straightLineEquation(135, kMaxAngularVel, 0, 0);
                }
                break;
            case 2:
                if (abs(bearing(dest_orientation, current_orientation)) > 5)
                {
                    velocity.linear.x = 0;
                    velocity.angular.z = -(abs(bearing(dest_orientation, current_orientation)) * bearing_equation[0] + bearing_equation[1]);
                    publishVel(velocity);
                }
                else
                {

                    ROS_INFO_STREAM("No Aruco Tag found on the Right...");
                    ROS_INFO_STREAM("Search Pattern: Hard Turning Left");
                    dest_orientation = current_orientation + 90;
                    if (dest_orientation > 360)
                        dest_orientation = dest_orientation - 360;
                    search_pattern_function = 3;
                    velocity.linear.x = 0;
                    velocity.angular.z = 0;
                    publishVel(velocity);
                    bearing_equation = equation_generation.straightLineEquation(270, kMaxAngularVel, 0, 0);
                }
                break;
            case 3:
                if (abs(bearing(dest_orientation, current_orientation)) > 5)
                {
                    velocity.linear.x = 0;
                    velocity.angular.z = (abs(bearing(dest_orientation, current_orientation)) * bearing_equation[0] + bearing_equation[1]);
                    publishVel(velocity);
                }
                else
                {
                    ROS_INFO_STREAM("No Aruco found on the Left...");
                    ROS_INFO_STREAM("Adding an offset...");
                    ROS_INFO("Current orientation  %f", current_orientation);

                    dest_orientation = current_orientation + 135 + getAngOffset();
                    ROS_INFO("Target orientation  %f", dest_orientation);
                    if (dest_orientation > 360)
                        dest_orientation = dest_orientation - 360;
                    search_pattern_function = 4;
                    velocity.linear.x = 0;
                    velocity.angular.z = 0;
                    publishVel(velocity);
                    bearing_equation = equation_generation.straightLineEquation(135, kMaxAngularVel, 0, 0);
                }
                break;
            case 4:
                if (abs(bearing(dest_orientation, current_orientation)) > 5)
                {
                    velocity.linear.x = 0;
                    velocity.angular.z = -(abs(bearing(dest_orientation, current_orientation)) * bearing_equation[0] + bearing_equation[1]);
                    publishVel(velocity);
                }
                else
                {
                    ROS_INFO_STREAM("Offset Reached...");
                    ROS_INFO_STREAM(getAngOffset());
                    setAngOffset(getAngOffset()+5);
                    search_pattern_function = 0;
                    velocity.linear.x = 0;
                    velocity.angular.z = 0;
                    publishVel(velocity);
                }
                break;
            }
            break;
        case kTerminate:
            setSearchPatternFlag(false);
            velocity.linear.x = 0;
            velocity.angular.z = 0;
            publishVel(velocity);
            break;
        }
    };

   
    void SensorCallback::obstacleAvoidance()
    {
        // ROS_INFO("here");
        velocity.linear.x = 0;//obs_avoid_linear[0] * obs_x + obs_avoid_linear[1];
        if (abs(obs_y)>kRoverBreadth/2+0.1){
            velocity.linear.x=0.5;
        }
        if(obs_y <= 0)
            velocity.angular.z = -1;//obs_avoid_angular[0] * abs(obs_y) + obs_avoid_angular[1];
        else if(obs_y > 0)
            velocity.angular.z = 1;//(obs_avoid_angular[0] * abs(obs_y) + obs_avoid_angular[1]);
        publishVel(velocity);
    };
    void SensorCallback::objectFollowing()
    {
        if (aruco_x >= 2)
            velocity.linear.x = kMaxLinearVel*0.8;
        else
            velocity.linear.x = obj_follow_linear[0] * aruco_x + obj_follow_linear[0];
        if (aruco_y >= 2)
            velocity.angular.z = -kMaxAngularVel;
        else if (aruco_y < -2)
            velocity.angular.z = kMaxAngularVel;
        else if (aruco_y > 0)
            velocity.angular.z = -(obj_follow_angular[0] * abs(aruco_y) + obj_follow_angular[0]);
        else if (aruco_y < 0)
            velocity.angular.z = obj_follow_angular[0] * abs(aruco_y) + obj_follow_angular[0];

        publishVel(velocity);
    };

    void SensorCallback::coordinateFollowing()
    {
        // ROS_INFO_STREAM("Angle to turn: " << coordinate_bearing);
        if (dest_distance > 5)
            velocity.linear.x = kMaxLinearVel / M_PI_2 * pow((abs(coordinate_bearing) - M_PI_2), 2);

        // Parabolic function: linear distance vs linear velocity

        else if (dest_distance <= 5)
            velocity.linear.x = 0.5 * pow(dest_distance, 2) / pow(M_PI_2, 2);

        // Angular Velocity Calculaton

        // Constant angular velocity till rover reaches gp_angle_buffer

        if (abs(coordinate_bearing) > M_PI_2)
            velocity.angular.z = kMaxAngularVel;

        // Parabolic function: bearing vs angular velocity

        else if (abs(coordinate_bearing) <= M_PI_2)
            velocity.angular.z = kMaxAngularVel * pow(coordinate_bearing, 2) / pow(M_PI_2, 2);

        // Taking the shortest turn 

        if (coordinate_bearing < 0)
			velocity.angular.z = -velocity.angular.z;
		else if (coordinate_bearing >= 0)
			velocity.angular.z = velocity.angular.z;

        publishVel(velocity);
    };

    void SensorCallback::setSearchPatternFlag (bool flag)
    {
        search_pattern_flag = flag;
    }

    bool SensorCallback::getSearchPatternFlag ()
    {
        return search_pattern_flag;
    }
}

#endif
