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
    int wd = 200;
    ros::param::get("~width", wd);
    im_ = cv::Mat(cv::Size(wd, wd), CV_32FC1, cv::Scalar::all(0.01));
    desired_sum_ = cv::sum(im_)[0];
    kernel_ = cv::Mat(cv::Size(3, 3), CV_32FC1, cv::Scalar::all(0.0));
    {
      float straight = 2.0;
      float corner = 1.4;
      kernel_.at<float>(0, 0) = corner;
      kernel_.at<float>(0, 1) = straight;
      kernel_.at<float>(0, 2) = corner;
      kernel_.at<float>(1, 0) = straight;
      kernel_.at<float>(1, 1) = 0.0;
      kernel_.at<float>(1, 2) = straight;
      kernel_.at<float>(2, 0) = corner;
      kernel_.at<float>(2, 1) = straight;
      kernel_.at<float>(2, 2) = corner;
      // normalize
      kernel_ /= cv::sum(kernel_)[0];
    }

    cv::namedWindow("grid");
    cv::setMouseCallback("grid", onMouse, this);

    lbutton_down_ = false;
    sc_ = 1.0;
      sc_ = 800.0 / wd;

    ros::param::get("~scale", sc_);

    while (ros::ok())
    {
      cv::Mat im_big;
      cv::resize(im_, im_big, cv::Size(0, 0), sc_, sc_, cv::INTER_NEAREST);
      cv::imshow("grid", im_big * 20.0);
      ros::Duration(0.05).sleep();
      int key = cv::waitKey(10);
      // if (key == 'n')
        update();
    }
  }

  bool isUnused(const int dx, const int dy)
  {
    const bool out_of_bounds = (dy < 0) || (dx < 0) || (dy >= im_.rows) || (dx >= im_.cols);
    if (out_of_bounds)
      return true;
    const bool is_wall = im_.at<float>(dy, dx) < 0;
    return is_wall;
  }

  void update()
  {
    const float ktot = cv::sum(kernel_)[0];
    cv::Point center = cv::Point(im_.cols / 2, im_.rows / 2);
    if (count_ % 40 == 0)
      cv::circle(im_, center, 4.0, cv::Scalar(10.0), -1);
    else if (count_ % 40 == 2)
      cv::circle(im_, center, 4.0, cv::Scalar(0.0), -1);

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
            if (isUnused(dx, dy))
            {
              unused += ksrc;
              ++num_unused;
            }
          }
        }

        const float div = (ktot - unused);
        float unused_scale = 1.0;
        if ((div != 0) && (num_unused > 0))
        {
          unused_scale = ktot / div;
          ROS_DEBUG_STREAM(y << " " << x << " " << unused_scale << ", "
              << ktot << " "
              << unused << " " << div << " " << num_unused);
        }

        // second pass to actuall disperse kernel 
        for (size_t ky = 0; ky < kernel_.rows; ++ky)
        {
          for (size_t kx = 0; kx < kernel_.cols; ++kx)
          {
            const float ksrc = kernel_.at<float>(ky, kx);
            const int dy = y + ky - hkht;
            // ROS_INFO_STREAM(dy << " = " << y << " + " << ky << " - " << hkht);
            const int dx = x + kx - hkwd;
            if (isUnused(dx, dy))
            {
            }
            else
            {
              // ROS_INFO_STREAM(dy << " " << dx << " " << src << " " << ksrc);
              im2.at<float>(dy, dx) += src * ksrc * unused_scale;
            }
          }
        }


      }
    }

    // this has some oscillation problems
    im_ = im2;  // * 0.9 * desired_sum_ / cv::sum(im2)[0];

    ++count_;
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

    if (event == cv::EVENT_RBUTTONDOWN)
    {
      ROS_INFO_STREAM("right button down");
      rbutton_down_ = true;
    }
    else if (event == cv::EVENT_RBUTTONUP)
    {
      ROS_INFO_STREAM("right button up");
      rbutton_down_ = false;
    }

    if (rbutton_down_)
    {
      cv::circle(im_, cv::Point(x / sc_ - 3, y / sc_), 7.0, cv::Scalar(0.0),
          -1);
      cv::circle(im_, cv::Point(x / sc_ + 1, y / sc_), 4.0, cv::Scalar(2.0),
          -1);
    }

  }

private:
  ros::NodeHandle nh_;
  cv::Mat im_;
  cv::Mat kernel_;
  bool lbutton_down_;
  bool rbutton_down_;
  float sc_;
  float desired_sum_;
  int count_;
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
