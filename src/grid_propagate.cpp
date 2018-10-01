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
    int wd = 32;
    ros::param::get("~width", wd);
    im_ = cv::Mat(cv::Size(wd, wd), CV_32FC1, cv::Scalar::all(0.5));
    desired_sum_ = cv::sum(im_)[0];
    kernel_ = cv::Mat(cv::Size(3, 3), CV_32FC1, cv::Scalar::all(0.0));
    kernel_.at<float>(0, 0) = 1.0;
    kernel_.at<float>(0, 1) = 2.0;
    kernel_.at<float>(0, 2) = 1.0;
    kernel_.at<float>(1, 0) = 2.0;
    kernel_.at<float>(1, 1) = 0.1;
    kernel_.at<float>(1, 2) = 2.0;
    kernel_.at<float>(2, 0) = 1.0;
    kernel_.at<float>(2, 1) = 2.0;
    kernel_.at<float>(2, 2) = 1.0;
    kernel_ /= cv::sum(kernel_)[0];

    cv::namedWindow("grid");
    cv::setMouseCallback("grid", onMouse, this);

    lbutton_down_ = false;
    sc_ = 800 / wd;
    ros::param::get("~scale", sc_);

    while (ros::ok())
    {
      cv::Mat im_big;
      cv::resize(im_, im_big, cv::Size(0, 0), sc_, sc_, cv::INTER_NEAREST);
      cv::imshow("grid", im_big);
      ros::Duration(0.05).sleep();
      int key = cv::waitKey(10);
      // if (key == 'n')
        update();
    }
  }

  void update()
  {
    const float ktot = cv::sum(kernel_)[0];

    cv::Mat im2(im_.size(), im_.type(), cv::Scalar::all(0.0));
    for (size_t y = 0; y < im_.rows; ++y)
    {
      for (size_t x = 0; x < im_.cols; ++x)
      {
        const float src = im_.at<float>(y, x);
        if (src < 0)
        {
          im2.at<float>(y, x) = src;
          continue;
        }
        int hkht = (kernel_.cols - 1) / 2;
        int hkwd = (kernel_.rows - 1) / 2;
        // TODO(lucasw) not very efficient

        // first pass just to accumulate unused
        float unused = 0.0;
        int num_unused = 0;
        for (size_t ky = 0; ky < kernel_.rows; ++ky)
        {
          for (size_t kx = 0; kx < kernel_.cols; ++kx)
          {
            const float ksrc = kernel_.at<float>(ky, kx);
            const int dy = y + ky - hkht;
            const int dx = x + kx - hkwd;
            if ((dy < 0) || (dx < 0) || (dy >= kernel_.rows) || (dx > kernel_.cols) ||
                (im_.at<float>(dy, dx) < 0))
            {
              unused += ksrc;
              ++num_unused;
            }
          }
        }

        // TODO(lucasw) this should scale differently?
        const float num_div = kernel_.rows * kernel_.cols - num_unused;
        float unused_offset = 0;
        if (num_div != 0)
          unused_offset = unused / num_div;

        // second pass to actuall disperse kernel 
        for (size_t ky = 0; ky < kernel_.rows; ++ky)
        {
          for (size_t kx = 0; kx < kernel_.cols; ++kx)
          {
            const float ksrc = kernel_.at<float>(ky, kx);
            const int dy = y + ky - hkht;
            // ROS_INFO_STREAM(dy << " = " << y << " + " << ky << " - " << hkht);
            const int dx = x + kx - hkwd;
            if ((dy < 0) || (dx < 0) || (dy >= im_.rows) || (dx >= im_.cols) ||
                (im_.at<float>(dy, dx) < 0))
            {
            }
            else
            {
              // ROS_INFO_STREAM(dy << " " << dx << " " << src << " " << ksrc);
              im2.at<float>(dy, dx) += src * ksrc + unused_offset;
            }
          }
        }


      }
    }

    // this has some oscillation problems
    im_ = im2;  // * 0.9 * desired_sum_ / cv::sum(im2)[0];
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
      // TODO(lucasw) move to update, store list of points here
      im_.at<float>(y / sc_, x / sc_) = -1.0;
    }
  }

private:
  ros::NodeHandle nh_;
  cv::Mat im_;
  cv::Mat kernel_;
  bool lbutton_down_;
  float sc_;
  float desired_sum_;
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
