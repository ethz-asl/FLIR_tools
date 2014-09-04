/*
 *
 * Copyright (c) 2013, Simon Lynen, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2013, Stefan Leutenegger, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2014, Pascal Gohl, ASL, ETH Zurich, Switzerland
 * You can contact the authors at <firstname.lastname at ethz dot ch>
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <fstream>
#include <iostream>
#include <list>
#include <iomanip>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>

#define ROS_MIN_MAJOR 1
#define ROS_MIN_MINOR 8
#define ROS_MIN_PATCH 16

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/CvBridge.h>
#endif
#include <falsecolor.h>

class ImagePublisher {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;

  ImagePublisher():it_(nh_) {
    std::cout<<"Advertising /ir_image/image_out"<<std::endl;
    image_pub_ = it_.advertise("/ir_image/image_out", 10);
    seq_ = 0;
  }
  ~ImagePublisher() {
    delete inst_;
    inst_ = NULL;
  }

 public:
  static ImagePublisher* Instance() {
    if (inst_ == NULL) {
      inst_ = new ImagePublisher();
    }
    assert(inst_);
    return inst_;
  }

  void publish(cv::Mat image){
    if(image_pub_.getNumSubscribers()){
      cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
      cv_ptr->encoding = "bgr8";
      cv_ptr->header.seq = seq_++;
      cv_ptr->header.stamp = ros::Time::now();
      cv_ptr->image = image;
      sensor_msgs::ImagePtr msg = cv_ptr->toImageMsg();
      image_pub_.publish(msg);
    }
  }

  static ImagePublisher* inst_;
  int seq_;
};

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
#else
  sensor_msgs::CvBridge bridge;
#endif
  try {

    // get image

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(msg, "mono16");
    cv::Mat img = ptr->image;
#else
    cv::Mat img = bridge.imgMsgToCv(msg, "mono16");
#endif

    //convert 8 bit and false color conversion
    static bool dofalsecolor = true;
    static bool doTempScaling = false;

    cv::Mat img_mono8_ir, img_mono8;
    img_mono8_ir.create(img.rows, img.cols, CV_8UC1);

    converter_16_8::Instance().convert_to8bit(img, img_mono8_ir, doTempScaling);
    //converter_16_8::Instance().toneMapping(img, img_mono8_ir);

    if (dofalsecolor) {
      convertFalseColor(img_mono8_ir, img_mono8, palette::False_color_palette4, doTempScaling,
                        converter_16_8::Instance().getMin(), converter_16_8::Instance().getMax());
    } else {
      img_mono8 = img_mono8_ir;
    }

    // display
    cv::Mat fullImg(img.rows, img.cols, CV_8UC3);
    cv::Mat dst_tl = fullImg(cv::Rect(0, 0, img.cols, img.rows));
    cv::Mat img_monorgb;
    if(img_mono8.type() == CV_8UC1){
      cv::cvtColor(img_mono8, img_monorgb, CV_GRAY2BGR);
      img_monorgb.copyTo(dst_tl);
    }else{
      img_mono8.copyTo(dst_tl);
    }

    ImagePublisher::Instance()->publish(fullImg);

  }

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
  catch (cv_bridge::Exception& e)
#else
  catch (sensor_msgs::CvBridgeException& e)
#endif
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ir_converter");
  std::string path = ros::package::getPath("ir_viewer");

  ImagePublisher::Instance();

  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
  ros::spin();

  return 0;
}

ImagePublisher* ImagePublisher::inst_ = NULL;
