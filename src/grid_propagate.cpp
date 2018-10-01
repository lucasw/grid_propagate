// Lucas Walter
// October 2018

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>

// class GridPropagate;
static void onMouse(int event, int x, int y, int flags, void* user_data);

class GridPropagate
{
public:
  GridPropagate()
  {
    int wd = 300;
    ros::param::get("~width", wd);
    im_ = cv::Mat(cv::Size(wd, wd), CV_32FC1, cv::Scalar::all(1.0));

    cv::namedWindow("grid");
    cv::setMouseCallback("grid", onMouse, this);

    lbutton_down_ = false;
    sc_ = 3.0;

    while (ros::ok())
    {
      cv::Mat im_big;
      cv::resize(im_, im_big, cv::Size(0, 0), sc_, sc_, cv::INTER_NEAREST);
      cv::imshow("grid", im_big);
      cv::waitKey(10);
      ros::Duration(0.05).sleep();
    }
  }

  void mouseEvent(const int event, const int x, const int y, const int flags)
  {
    ROS_DEBUG_STREAM(event << " " << x << " " << y << " " << flags);
    if (event == cv::EVENT_LBUTTONDOWN)
    {
      ROS_INFO_STREAM("left button down");
      lbutton_down_ = true;
    }
    else if (event == cv::EVENT_LBUTTONUP)
    {
      ROS_INFO_STREAM("left button up");
      lbutton_down_ = false;
    }

    if (lbutton_down_)
    {
      im_.at<float>(y / sc_, x / sc_) = 0.0;
    }
  }

private:
  ros::NodeHandle nh_;
  cv::Mat im_;
  bool lbutton_down_;
  float sc_;
};

static void onMouse(int event, int x, int y, int flags, void* user_data)
{
  GridPropagate* gp = static_cast<GridPropagate*>(user_data);

  gp->mouseEvent(event, x, y, flags);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_propate");
  GridPropagate grid_propagate;
}
