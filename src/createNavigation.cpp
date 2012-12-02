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

void setRobotCtrlCallBack(const std_msgs::UInt8& ctrl)
{
  if(sysState >= robotCtrlOn && sysState < missionComplete)
  {
    switch(sysState)
    {
      case robotCtrlOn:
        if(ctrl == 0);
        break;
    }
  }
}

void robotHeadingCallBack(const std_msgs::Float32& angle)
{

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
  if(sysState == startPrgm)
  {
    imgCount++;
    if(imgCount == 15)
    {
      sysState = pictureSentToUser;
      imgCount = 0;

      //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
      cv_bridge::CvImagePtr cv_ptr;
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

  //Create the ROS node handle
  ros::NodeHandle nh;
  //Create an ImageTransport instance, initializing it with our NodeHandle.
  image_transport::ImageTransport it(nh);

  //Initialize subscribers
  image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
  ros::Subscriber coordSub = nh.subscribe("dest_coord", 1, coordCallBack);
  ros::Subscriber manAuto = nh.subscribe("RobotCtrlMode", 10, setRobotCtrlCallBack);
  ros::Subscriber angleSub = nh.subscribe("robot_angle_variation", 10, robotHeadingCallBack);

  //Initialize the publishers
  pub = it.advertise("userImage", 1);

  //Initialize the system state
  sysState = startPrgm;
  navCtrlMode = navOff;

  ros::spin();
  //ROS_INFO is the replacement for printf/cout.
  ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");

}
