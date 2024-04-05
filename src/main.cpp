#include <iostream>
#include "opencv2/opencv.hpp"
// #include <nlohmann/json.hpp>
#include <fstream>
#include <gtkmm/application.h>
#include <gtkmm/button.h>
#include <gtkmm/window.h>

extern "C" {
  #include "apriltag.h"
  #include "tag36h11.h"
  #include "common/getopt.h"
}

// using json = nlohmann::json;

struct Tag {
  int id;
  double pitch, yaw, roll, i, j, k;
  bool isSeed;
  bool isGauged;
};

class Window : public Gtk::Window {
  public:
    Window() : button("Hello world") {
      set_title("Basic application");
      set_default_size(200, 200);
      button.signal_clicked().connect(sigc::mem_fun(*this, &Window::on_button_clicked));
      set_child(button);
    }
    virtual ~Window() {};
  protected:
    void on_button_clicked() {
      std::cout << "Hello world" << std::endl;
    }

    Gtk::Button button;
};

// Largely based on example at https://github.com/AprilRobotics/apriltag/blob/master/example/opencv_demo.cc

/*int main(int argc, char* argv[]) {
  
  // Argument for ID of webcam to use
  getopt_t* getopt = getopt_create();
  getopt_add_int(getopt, 'c', "camera", "0", "camera ID");
  getopt_add_string(getopt, 'C', "config", "config.fmap", "path to field map file");

  if (!getopt_parse(getopt, argc, argv, 1)) exit(0);

  // Load field tag map from configuration file
  std::ifstream config(getopt_get_string(getopt, "config"));

  // Camera initialization
  cv::VideoCapture cap(getopt_get_int(getopt, "camera"));
  if (!cap.isOpened()) {
    std::cerr << "Couldn't open video capture device" << std::endl;
  }

  apriltag_family_t* tf = tag36h11_create(); // 36h11 tags used in FRC
  apriltag_detector_t* td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);

  // TODO - tune
  td->quad_decimate = 2.0;
  td->quad_sigma = 0; // blur
  td->nthreads = 1;
  td->debug = 0;
  td->refine_edges = 1;

  cv::Mat frame, gray;

  auto app = Gtk::Application::create();
  // app.

  while (true) {
    cap >> frame;
    cvtColor(frame, gray, cv::COLOR_BGR2GRAY); // black and white image
    
    // Make an image_u8_t header for the Mat data
    image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};

    zarray_t* detections = apriltag_detector_detect(td, &im);

    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      line(frame, cv::Point(det->p[0][0], det->p[0][1]),
        cv::Point(det->p[1][0], det->p[1][1]),
        cv::Scalar(0, 0xff, 0), 2);
      line(frame, cv::Point(det->p[0][0], det->p[0][1]),
        cv::Point(det->p[3][0], det->p[3][1]),
        cv::Scalar(0, 0, 0xff), 2);
      line(frame, cv::Point(det->p[1][0], det->p[1][1]),
        cv::Point(det->p[2][0], det->p[2][1]),
        cv::Scalar(0xff, 0, 0), 2);
      line(frame, cv::Point(det->p[2][0], det->p[2][1]),
        cv::Point(det->p[3][0], det->p[3][1]),
        cv::Scalar(0xff, 0, 0), 2);

      std::stringstream ss;
      ss << det->id;
      std::string text = ss.str();
      int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
      double fontscale = 1.0;
      int baseline;
      cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2, &baseline);
      putText(frame, text, cv::Point(det->c[0]-textsize.width/2,
        det->c[1]+textsize.height/2),
        fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
    }
    apriltag_detections_destroy(detections);
    // imshow("Tag Detections", frame);
    if (cv::waitKey(30) >= 0) break;
  }

  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
  getopt_destroy(getopt);

  return 0;
}*/

int main(int argc, char** argv) {
  auto app = Gtk::Application::create();
  return app->make_window_and_run<Window>(argc, argv);
}
