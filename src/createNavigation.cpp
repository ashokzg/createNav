//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include "../../brownCreate/brown_drivers/irobot_create_2_1/msg_gen/cpp/include/irobot_create_2_1/SensorPacket.h"


void setBotToStop(geometry_msgs::Twist& botVel);

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";

static int imgCount;

/* Monitor the state of the program
 * 0 => Start of program
 * 1 => Camera node started, images obtained
 */
typedef enum
{
  startPrgm,
  camStarted,
  sendPicture,
  pictureSentToUser,
  coordReceived,
  robotCtrlOn,
  destLost,
  obstacleDetected,
  robotCtrlOff,
  missionComplete,
} createState_t;

typedef enum
{
  navOff,
  autoNav,
  manualNav
}robotNavCtrlMode_t;

static createState_t sysState;
static robotNavCtrlMode_t navCtrlMode;


//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
ros::Publisher robotCmdPub;

/* Publishes a UInt8 message with the following enum
 * 0: Idle
 * 1: Moving to Destination
 * 2: Lost Destination
 * 3: Reached Destination
 * 4: Mission Complete
 */
ros::Publisher robotStatusPub;

void sensorCallBack(const irobot_create_2_1::SensorPacket& sensors)
{
  geometry_msgs::Twist botVel;
  //Ultrasonic sensor signal should be monitored for obstacles
  if(sensors.user_analog_signal < 80)
  {
    setBotToStop(botVel);
    ROS_INFO("WARNING: OBSTACLE");
    ROS_INFO("The sensor distance is %d",sensors.user_analog_signal);
    robotCmdPub.publish(botVel);
    sysState = obstacleDetected;
  }
}

void setRobotCtrlCallBack(const std_msgs::UInt8& ctrl)
{
  //Check if the received value is within range and set the appropriate mode
  if(ctrl.data >= (char)navOff && ctrl.data <= (char)manualNav)
  {
    ROS_INFO("Mode changed to %d (0:Off 1:Auto 2:Manual)", ctrl.data);
    navCtrlMode = (robotNavCtrlMode_t)ctrl.data;
  }
}

void manualCtrlCallBack(const geometry_msgs::Twist& manCmdVel)
{
  if(navCtrlMode == manualNav)
    robotCmdPub.publish(manCmdVel);
}

void robotAreaCallBack(const std_msgs::Float32& area)
{
  if(area.data > (float)(320*240))
  {
    printf("***DESTINATION REACHED BASED ON AREA");
    //TBD implement a publisher for robot state
  }

}

void robotStatusCallBack(const std_msgs::UInt8& robot_status)
{
  std_msgs::UInt8 pubState;
  switch(robot_status.data)
  {
    //Image processing IDLE
    case 0:
      break;
    //Image processing is tracking the destination
    case 1:
      pubState.data = 1;
      robotStatusPub.publish(pubState);
      break;
    //Image processing has lost destination
    case 2:
      pubState.data = 2;
      robotStatusPub.publish(pubState);
      geometry_msgs::Twist botVel;
      setBotToStop(botVel);
      robotCmdPub.publish(botVel);
      navCtrlMode = navOff;
      sysState = startPrgm;
      /* Switching to manual navigation because the destination is lost*/
      ROS_INFO("***ERROR: DESTINATION LOST. RESETTING TO INITIAL STATE TO RESUME OPERATION***");
      ROS_INFO("***ERROR: STOPPING ROBOT MOTION***");
      break;
  }

//  if(robot_status.data == 3)
//  {
//    ROS_INFO("---SUCCESS: REACHED DESTINATION, BITCH!!---");
//    sysState = robotCtrlOff;
//    geometry_msgs::Twist botVel;
//    setBotToStop(botVel);
//    robotCmdPub.publish(botVel);
//  }
}

void setBotToStop(geometry_msgs::Twist& botVel)
{
  botVel.linear.x = 0;
  botVel.linear.y = 0;
  botVel.linear.z = 0;
  botVel.angular.x = 0;
  botVel.angular.y = 0;
  botVel.angular.z = 0;
}

void robotHeadingCallBack(const std_msgs::Float32& angle)
{
  if(sysState == robotCtrlOn && navCtrlMode == autoNav)
  {
    ROS_INFO("Angle is %f", angle.data);
    geometry_msgs::Twist botVel;
    if(angle.data > 0.1)
    {
        //Linear x is positive for forward direction
        botVel.linear.x = 0.3;
        //Angular z is negative for right
        botVel.angular.z = -0.5;
    }
    else if(angle.data < -0.1)
    {
        //Linear x is positive for forward direction
        botVel.linear.x = 0.3;
        //Angular z is negative for right
        botVel.angular.z = 0.5;
    }
    else
    {
        //Linear x is positive for forward direction
        botVel.linear.x = 0.3;
    }
    robotCmdPub.publish(botVel);
  }
}

void coordCallBack(const sensor_msgs::RegionOfInterest& roi)
{
  if(sysState == pictureSentToUser)
  {
    ROS_INFO("DESTINATION RECEIVED BY ROBOT. WILL START NAVIGATION");
    sysState = robotCtrlOn;
    navCtrlMode = autoNav;
  }
  else
  {
    ROS_INFO("***WARNING: COORDINATES RECEIVED AT INAPPROPRIATE TIME***");
  }
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
  //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
  cv_bridge::CvImagePtr cv_ptr;

  imgCount++;
  try
  {
      //Always copy, returning a mutable CvImage
      //OpenCV expects color images to use BGR channel order.
      cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
      //if there is an error during conversion, display it
      ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
      return;
  }

  //Display the image using OpenCV
  cv::imshow(WINDOW, cv_ptr->image);
  //Add some delay in milliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
  cv::waitKey(3);

  //if(sysState == startPrgm)
  if(sysState < robotCtrlOn)
  {
    if(imgCount >= 15)
    {
      imgCount = 0;
      sysState = pictureSentToUser;

      /**
      * The publish() function is how you send messages. The parameter
      * is the message object. The type of this object must agree with the type
      * given as a template parameter to the advertise<>() call, as was done
      * in the constructor in main().
      */
      //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
      pub.publish(cv_ptr->toImageMsg());
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "create_navigation");
  ROS_INFO("Node started");

  //Create the ROS node handle
  ros::NodeHandle nh;
  //Create an ImageTransport instance, initializing it with our NodeHandle.
  image_transport::ImageTransport it(nh);

  //Initialize subscribers
  //Published by camera node
  image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);

  //Published by UI node
  ros::Subscriber coordSub  = nh.subscribe("dest_coord", 1, coordCallBack);
  ros::Subscriber manAuto   = nh.subscribe("RobotCtrlMode", 10, setRobotCtrlCallBack);
  ros::Subscriber manCmd    = nh.subscribe("man_cmd_vel", 100, manualCtrlCallBack);

  //Published by computer vision node
  ros::Subscriber angleSub  = nh.subscribe("robot_angle", 10, robotHeadingCallBack);
  ros::Subscriber statusSub = nh.subscribe("improc_state", 1, robotStatusCallBack);
  ros::Subscriber destAreaSub = nh.subscribe("dest_area", 10, robotAreaCallBack);

  //Published by irobot_create node
  ros::Subscriber createSensorSub = nh.subscribe("sensorPacket", 10, sensorCallBack);

  //---------------------------
  //Initialize the publishers
  //Subscribed by irobot node
  robotCmdPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

  //Subscribed by UI node
  pub = it.advertise("camera/userImage", 1);
  robotStatusPub = nh.advertise<std_msgs::UInt8>("robot_state", 10);

  //Initialize the system state
  sysState = startPrgm;
  navCtrlMode = navOff;

  //Publish initial values
  std_msgs::UInt8 pubState;
  geometry_msgs::Twist botVel;
  setBotToStop(botVel);
  pubState.data = 0;

  robotStatusPub.publish(pubState);
  robotCmdPub.publish(botVel);

  ros::spin();
  //ROS_INFO is the replacement for printf/cout.
  ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
}
