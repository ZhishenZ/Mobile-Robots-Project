#include <memory>
#include <unistd.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <arp/Autopilot.hpp>

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    // -- for later use
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
  }

  bool getLastImage(cv::Mat& image)
  {
    std::lock_guard<std::mutex> l(imageMutex_);
    if (lastImage_.empty())
      return false;
    image = lastImage_.clone();
    lastImage_ = cv::Mat();  // clear, only get same image once.
    return true;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    // -- for later use
  }

 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // setup inputs
  Subscriber subscriber;
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);

  // set up autopilot
  arp::Autopilot autopilot(nh);

  // setup rendering
  SDL_Event event;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window * window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED,
                                         SDL_WINDOWPOS_UNDEFINED, 640, 360, 0);
  SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
  SDL_Texture * texture;

  // enter main event loop
  std::cout << "===== Hello AR Drone ====" << std::endl;
  cv::Mat image;
  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
      break;
    }
  
    // render image, if there is a new one available
    if(subscriber.getLastImage(image)) {
        cv::putText(image, //target image
              "forward: up arrow; backward: down arrow; left: left arrow; right: right arrow", //text
              cv::Point(10, 10), //top-left position
              cv::FONT_HERSHEY_DUPLEX,
              0.5,
              CV_RGB(118, 185, 0), //font color
              0.5);
      cv::putText(image, //target image
              "up: w; down: s; rotateleft: a; rotateright: d", //text
              cv::Point(10, 30), //top-left position
              cv::FONT_HERSHEY_DUPLEX,
              0.5,
              CV_RGB(118, 185, 0), //font color
              0.5);
      cv::putText(image, //target image
              "takeoff: t; land: l; shutting off: esc; calibration: c", //text
              cv::Point(10, 50), //top-left position
              cv::FONT_HERSHEY_DUPLEX,
              0.5,
              CV_RGB(118, 185, 0), //font color
              0.5);
      // TODO: add overlays to the cv::Mat image, e.g. text
      
      // https://stackoverflow.com/questions/22702630/converting-cvmat-to-sdl-texture
      // I'm using SDL_TEXTUREACCESS_STREAMING because it's for a video player, you should
      // pick whatever suits you most: https://wiki.libsdl.org/SDL_TextureAccess
      // remember to pick the right SDL_PIXELFORMAT_* !
      texture = SDL_CreateTexture(
          renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, image.cols, image.rows);
      SDL_UpdateTexture(texture, NULL, (void*)image.data, image.step1());
      SDL_RenderClear(renderer);
      SDL_RenderCopy(renderer, texture, NULL, NULL);
      SDL_RenderPresent(renderer);
      // cleanup (only after you're done displaying. you can repeatedly call UpdateTexture without destroying it)
      SDL_DestroyTexture(texture);

    }
    //Multiple Key Capture Begins
    const Uint8 *state = SDL_GetKeyboardState(NULL);

    // check states!
    auto droneStatus = autopilot.droneStatus();

    // command

    if (state[SDL_SCANCODE_ESCAPE]) {
      std::cout << "ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=" << droneStatus;
      bool success = autopilot.estopReset();
      if(success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_T]) {
      std::cout << "Taking off...                          status=" << droneStatus;
      bool success = autopilot.takeoff();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    if (state[SDL_SCANCODE_L]) {
      std::cout << "Going to land...                       status=" << droneStatus;
      bool success = autopilot.land();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_C]) {
      std::cout << "Requesting flattrim calibration...     status=" << droneStatus;
      bool success = autopilot.flattrimCalibrate();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    // TODO: process moving commands when in state 3,4, or 7
    // if (state[SDL_SCANCODE_UP]) {
    //   std::cout << "Moving forward...     status=" << droneStatus;
    //   bool success = autopilot.manualMove(0.5, 0, 0, 0);
    //   if (success) {
    //     std::cout << " [ OK ]" << std::endl;
    //   } else {
    //     std::cout << " [FAIL]" << std::endl;
    //   }
    // }

    // if (state[SDL_SCANCODE_DOWN]) {
    //   std::cout << "Moving backward...     status=" << droneStatus;
    //   bool success = autopilot.manualMove(-0.5, 0, 0, 0);
    //   if (success) {
    //     std::cout << " [ OK ]" << std::endl;
    //   } else {
    //     std::cout << " [FAIL]" << std::endl;
    //   }
    // }

    // if (state[SDL_SCANCODE_LEFT]) {
    //   std::cout << "Moving left...     status=" << droneStatus;
    //   bool success = autopilot.manualMove(0, 0.5, 0, 0);
    //   if (success) {
    //     std::cout << " [ OK ]" << std::endl;
    //   } else {
    //     std::cout << " [FAIL]" << std::endl;
    //   }
    // }

    // if (state[SDL_SCANCODE_RIGHT]) {
    //   std::cout << "Moving right...     status=" << droneStatus;
    //   bool success = autopilot.manualMove(0, -0.5, 0, 0);
    //   if (success) {
    //     std::cout << " [ OK ]" << std::endl;
    //   } else {
    //     std::cout << " [FAIL]" << std::endl;
    //   }
    // }

    // if (state[SDL_SCANCODE_W]) {
    //   std::cout << "Moving up...     status=" << droneStatus;
    //   bool success = autopilot.manualMove(0, 0, 0.5, 0);
    //   if (success) {
    //     std::cout << " [ OK ]" << std::endl;
    //   } else {
    //     std::cout << " [FAIL]" << std::endl;
    //   }
    // }

    // if (state[SDL_SCANCODE_S]) {
    //   std::cout << "Moving down...     status=" << droneStatus;
    //   bool success = autopilot.manualMove(0, 0, -0.5, 0);
    //   if (success) {
    //     std::cout << " [ OK ]" << std::endl;
    //   } else {
    //     std::cout << " [FAIL]" << std::endl;
    //   }
    // }

    // if (state[SDL_SCANCODE_A]) {
    //   std::cout << "Turning left...     status=" << droneStatus;
    //   bool success = autopilot.manualMove(0, 0, 0, 0.5);
    //   if (success) {
    //     std::cout << " [ OK ]" << std::endl;
    //   } else {
    //     std::cout << " [FAIL]" << std::endl;
    //   }
    // }

    // if (state[SDL_SCANCODE_D]) {
    //   std::cout << "Turning left...     status=" << droneStatus;
    //   bool success = autopilot.manualMove(0, 0, 0, -0.5);
    //   if (success) {
    //     std::cout << " [ OK ]" << std::endl;
    //   } else {
    //     std::cout << " [FAIL]" << std::endl;
    //   }
    // }
    
    // if (!state[SDL_SCANCODE_A
    //            || SDL_SCANCODE_D
    //            || SDL_SCANCODE_S
    //            || SDL_SCANCODE_W
    //            || SDL_SCANCODE_UP
    //            || SDL_SCANCODE_DOWN
    //            || SDL_SCANCODE_LEFT
    //            || SDL_SCANCODE_RIGHT]) {
    //   // ros::Rate rate(10);
    //   bool success = autopilot.manualMove(0, 0, 0, 0);
    //   // rate.sleep();
    // }

    if (state[SDL_SCANCODE_A]
               || state[SDL_SCANCODE_D]
               || state[SDL_SCANCODE_S]
               || state[SDL_SCANCODE_W]
               || state[SDL_SCANCODE_UP]
               || state[SDL_SCANCODE_DOWN]
               || state[SDL_SCANCODE_LEFT]
               || state[SDL_SCANCODE_RIGHT]) {
      // ros::Rate rate(10);
      double forward = 0;
      double left = 0;
      double up = 0;
      double rotateLeft = 0;

      if(state[SDL_SCANCODE_A]){rotateLeft = 0.5;}
      if(state[SDL_SCANCODE_D]){rotateLeft = -0.5;}
      if(state[SDL_SCANCODE_A] && state[SDL_SCANCODE_D]){rotateLeft = 0;}

      if(state[SDL_SCANCODE_W]){up = 0.5;}
      if(state[SDL_SCANCODE_S]){up = -0.5;}
      if(state[SDL_SCANCODE_W] && state[SDL_SCANCODE_S]){up = 0;}

      if(state[SDL_SCANCODE_UP]){forward = 0.5;}
      if(state[SDL_SCANCODE_DOWN]){forward = -0.5;}
      if(state[SDL_SCANCODE_UP] && state[SDL_SCANCODE_DOWN]){forward = 0;}

      if(state[SDL_SCANCODE_LEFT]){left = 0.5;}
      if(state[SDL_SCANCODE_RIGHT]){left = -0.5;}
      if(state[SDL_SCANCODE_LEFT] && state[SDL_SCANCODE_RIGHT]){left = 0;}

      
      

      std::cout << "Moving...     status=" << droneStatus;
      bool success = autopilot.manualMove(forward, left, up, rotateLeft);
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
      // rate.sleep();
    } 
    else{
      bool success = autopilot.manualMove(0, 0, 0, 0);
    }

  }

  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

