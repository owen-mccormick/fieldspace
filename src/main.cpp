#include <iostream>
#include <string>
// #include <regex>
#include <map>
#include <math.h>
#include "opencv2/opencv.hpp"
// #include <nlohmann/json.hpp>
// #include <fstream>
#include <gtkmm.h>
#include <gtkmm/application.h>
// #include <gtkmm/button.h>
#include <gtkmm/window.h>
#include <gtkmm/drawingarea.h>
#include <gdkmm/pixbuf.h>
#include <gdkmm/general.h>
#include <Eigen/Geometry>

extern "C" {
  #include "apriltag.h"
  #include "tag36h11.h"
  #include "apriltag_pose.h"
  #include "common/getopt.h"
}

// using json = nlohmann::json;

// TODO - be consistent about CamelCase vs underscore_naming

// Are top-level statements like this a good idea?
// Should be relocated inside of the Window class
cv::VideoCapture cap(0);
apriltag_family_t* tf = tag36h11_create(); // 36h11 tag family used in FRC
apriltag_detector_t* td = apriltag_detector_create();

struct TagPose {
  double x, y, theta;
};

// Subclass of entry box that only accepts numeric input (https://stackoverflow.com/questions/10279579/force-numeric-input-in-a-gtkentry-widget#10282948)
// TODO - this only works when inserting text from code; user can still enter letter and symbols
class NumericEntry : public Gtk::Entry {
  public:
    void on_insert_text(const Glib::ustring& text, int* position);
  protected:
    bool contains_only_numbers(const Glib::ustring& text);
};

void NumericEntry::on_insert_text(const Glib::ustring& text, int* position) {
  if (contains_only_numbers(text)) Gtk::Entry::on_insert_text(text, position);
}

bool NumericEntry::contains_only_numbers(const Glib::ustring& text) {
  for (int i = 0; i < text.length(); i++) {
    if(Glib::Unicode::isdigit(text[i]) == false) return false;
  }
  return true;
}

// FeedAndField: Represents an OpenCV camera feed and field map drawn in a gtkmm DrawingArea
class FeedAndField : public Gtk::DrawingArea {
  public:
    FeedAndField();
    // May be a more ergonomic way to pass this data around
    void setSeedID(int id);
    void setSeedX(double x);
    void setSeedY(double y);
    void setSeedRotate(double angle);
    int getSeedID();
    double getSeedX();
    double getSeedY();
    double getSeedRotate(); // in degrees?
    virtual ~FeedAndField();
  protected:
    void plotOnField(const Cairo::RefPtr<Cairo::Context>& cr, double xMeters, double yMeters, double theta, int fieldXPixels, int fieldYPixels, bool isSeed);
    void onDraw(const Cairo::RefPtr<Cairo::Context>& cr, int width, int height);
    // apriltag_family_t* tf;
    // apriltag_detector_t* td;
    const int fieldX = 485;
    const int fieldY = 2;
    const double fieldMetersX = 16.6;
    const double fieldMetersY = 8.2;
    const double tagSize = 0.1651; // Size of AprilTags in meters; should be 6.5 inches to meters according to the 2024 FRC game manual
    const double fx = 500; // TODO - tune / calibrate to camera; maybe accept these as arguments
    const int fy = 500;
    const int cx = 250;
    const int cy = 250;
    int seedID = 0;
    double seedX = 0;
    double seedY = 0;
    double seedRotate = 0;
    std::map<int, TagPose> tagPoses;
};

FeedAndField::FeedAndField() {
  tagPoses = std::map<int, TagPose>();
  apriltag_detector_add_family(td, tf);
  // TODO - tune
  td->quad_decimate = 2.0;
  td->quad_sigma = 0; // blur
  td->nthreads = 1;
  td->debug = 0;
  td->refine_edges = 1;
  set_draw_func(sigc::mem_fun(*this, &FeedAndField::onDraw));
}

// Setters and getters
void FeedAndField::setSeedID(int id) {
  if (id != seedID) {
    seedID = id;
    // Clear cached poses
    tagPoses.clear();
    tagPoses[seedID] = {seedX, seedY, seedRotate};
  }
}

void FeedAndField::setSeedX(double x) {
  if (x != seedX) {
    seedX = x;
    // Clear cached poses
    tagPoses.clear();
    tagPoses[seedID] = {seedX, seedY, seedRotate};
  }
}

void FeedAndField::setSeedY(double y) {
  if (y != seedY) {
    seedY = y;
    // Clear cached poses
    tagPoses.clear();
    tagPoses[seedID] = {seedX, seedY, seedRotate};
  }
}

void FeedAndField::setSeedRotate(double theta) {
  if (theta != seedRotate) {
    seedRotate = theta;
    // Clear cached poses
    tagPoses.clear();
    tagPoses[seedID] = {seedX, seedY, seedRotate};
  }
}

int FeedAndField::getSeedID() { return seedID; }
double FeedAndField::getSeedX() { return seedX; }
double FeedAndField::getSeedY() { return seedY; }
double FeedAndField::getSeedRotate() { return seedRotate; }

FeedAndField::~FeedAndField() {
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
}

// For labelling the dots that get drawn on the map by AprilTag ID
// From https://gnome.pages.gitlab.gnome.org/gtkmm-documentation/sec-drawing-text.html
// void FeedAndField::draw_text(const Cairo::RefPtr<Cairo::Context>& cr, int x, int y int rectangle_width, int rectangle_height, std::string text) {
  // Pango::FontDescription font;

  // font.set_family("Monospace");
  // font.set_weight(Pango::Weight::BOLD);

  // auto layout = create_pango_layout(text);

  // layout->set_font_description(font);

  // int text_width;
  // int text_height;

  // layout->get_pixel_size(text_width, text_height);

  // cr->move_to((rectangle_width-text_width) / 2, (rectangle_height-text_height) / 2);

  // layout->show_in_cairo_context(cr);
// }

void FeedAndField::plotOnField(const Cairo::RefPtr<Cairo::Context>& cr, double xMeters, double yMeters, double theta, int fieldXPixels, int fieldYPixels, bool isSeed) {
  double x = fieldX + xMeters * fieldXPixels / fieldMetersX;
  double y = fieldY + yMeters * fieldYPixels / fieldMetersY;
  cr->save();
  cr->arc(x, y, 10, 0, 2.0 * M_PI);
  if (isSeed) {
    cr->set_source_rgb(0, 0, 0.8);
  } else {
    cr->set_source_rgb(0, 0.8, 0);
  }
  cr->fill_preserve();
  cr->move_to(x, y);
  cr->line_to(x + 20 * cos(theta * M_PI / 180), y + 20 * sin(theta * M_PI / 180));
  cr->restore();
  cr->stroke();
}

void FeedAndField::onDraw(const Cairo::RefPtr<Cairo::Context>& cr, int width, int height) {
  cv::Mat frame, gray;
  cap >> frame;
  // frame = cv::imread("/home/omccormick/dev/fieldspace/resources/tagformats_web.png");
  cvtColor(frame, gray, cv::COLOR_BGR2GRAY); // black and white image

  // AprilTag detections
  // Make an image_u8_t header for the Mat data
  image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};

  zarray_t* detections = apriltag_detector_detect(td, &im);

  for (int i = 0; i < zarray_size(detections); i++) {
    // Draw detection outlines
    // Largely based on example at https://github.com/AprilRobotics/apriltag/blob/master/example/opencv_demo.cc
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
  // TODO - label plotted points on map with ID of corresponding tag
  // OpenCV drawing test
  // int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  // double fontscale = 1.0;
  // int baseline;
  // cv::Size textsize = cv::getTextSize("Hello", fontface, fontscale, 2, &baseline);
  // putText(frame, "Hello", cv::Point(50,
    // 50),
    // fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);

  // Convert from OpenCV types to image in GTK drawing area
  auto img = Gdk::Pixbuf::create_from_data(frame.data, Gdk::Colorspace::RGB, false, 8, frame.rows, frame.cols, frame.step);
  // set_content_height(img->get_height());
  // set_content_width(img->get_width());
  Gdk::Cairo::set_source_pixbuf(cr, img, 0, 0);
  cr->rectangle(2, 2, img->get_width(), img->get_height() * 2 / 3);
  cr->fill();

  // Field drawing: 2D field view updated with labeled dots for tag sightings
  // Field image from https://www.chiefdelphi.com/t/2024-crescendo-top-down-field-renders/447764
  // TODO - use glib-compile-resources to bake image into executable
  // Also probably don't need to read file from disk every tick
  auto field = Gdk::Pixbuf::create_from_file("resources/2160xDarkCroppedFixed.png");
  // Reize field image
  field = field->scale_simple(field->get_width() / 3.65, field->get_height() / 3.65, Gdk::InterpType::BILINEAR);
  Gdk::Cairo::set_source_pixbuf(cr, field, fieldX, fieldY);
  cr->rectangle(fieldX, fieldY, field->get_width(), field->get_height());
  cr->fill();

  // Graph seed tag
  plotOnField(cr, seedX, seedY, seedRotate, field->get_width(), field->get_height(), true);

  // Plot each tag on the map judging from the pose of a reference tag that already has a pose estimate, starting from the seed.
  // We need this because we have no idea where the camera is.
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *reference_det;
    zarray_get(detections, i, &reference_det);
    int reference_id = reference_det->id;
    for (int j = 0; j < zarray_size(detections); j++) {
      if (j != i) { //
        apriltag_detection_t *to_compare_det;
        zarray_get(detections, j, &to_compare_det);
        int to_compare_id = to_compare_det->id;
        
        if (tagPoses.count(reference_id) && to_compare_id != seedID) { // We have a pose for the reference tag
          // Do pose estimation with the reference and new tag
          apriltag_detection_info_t reference_info;
          reference_info.det = reference_det;
          reference_info.tagsize = tagSize;
          reference_info.fx = fx;
          reference_info.fy = fy;
          reference_info.cx = cx;
          reference_info.cy = cy;
          apriltag_pose_t reference_pose;
          estimate_tag_pose(&reference_info, &reference_pose);
          apriltag_detection_info_t to_compare_info;
          to_compare_info.det = to_compare_det;
          to_compare_info.tagsize = tagSize;
          to_compare_info.fx = fx;
          to_compare_info.fy = fy;
          to_compare_info.cx = cx;
          to_compare_info.cy = cy;
          apriltag_pose_t to_compare_pose;
          estimate_tag_pose(&to_compare_info, &to_compare_pose);
          tagPoses[to_compare_id] = {
            tagPoses[reference_id].x - reference_pose.t->data[0] + to_compare_pose.t->data[0],
            tagPoses[reference_id].y - reference_pose.t->data[2] + to_compare_pose.t->data[2],
            tagPoses[reference_id].theta
          }; 
          // std::cout << "ID " << to_compare_id << " yaw: " << to_compare_pose.R->data.eulerAngles(2, 1, 0) << std::endl;
        }
      }
    }
  }

  // Iterate through known points and plot, with the exception of the seed that has already been plotted above
  for (auto const& [id, pose] : tagPoses) {
    if (id != seedID) {
      plotOnField(cr, pose.x, pose.y, pose.theta, field->get_width(), field->get_height(), false);
    }
  }
  apriltag_detections_destroy(detections);
}

class Window : public Gtk::Window {
  public:
    Window();
  protected:
    Gtk::Grid grid;
    FeedAndField feedAndField;
    NumericEntry seedTagEntry, seedXEntry, seedYEntry, seedRotateEntry;
    bool periodic();
};

bool Window::periodic() {
  // std::cout << "Redraw...`" << std::endl;
  // Interpret input from text boxes
  try {
    feedAndField.setSeedID(std::stod(seedTagEntry.get_text()));
  } catch (std::exception e) {
    feedAndField.setSeedID(0);
    // std::cout << "Please enter an integer seed ID." << std::endl;
  }
  try {
    char prefix = seedXEntry.get_text()[0];
    if (prefix == '-') {
      feedAndField.setSeedX(-std::stod(seedXEntry.get_text().erase(0, 1)));
    } else {
      feedAndField.setSeedX(std::stod(seedXEntry.get_text()));
    }
  } catch (std::exception e) {
    feedAndField.setSeedX(0);
    // std::cout << "Please enter a numeric seed x coordinate." << std::endl;
  }
  try {
    char prefix = seedYEntry.get_text()[0];
    if (prefix == '-') {
      feedAndField.setSeedY(-std::stod(seedYEntry.get_text().erase(0, 1)));
    } else {
      feedAndField.setSeedY(std::stod(seedYEntry.get_text()));
    }
  } catch (std::exception e) {
    feedAndField.setSeedY(0);
    // std::cout << "Please enter a numeric seed y coordinate." << std::endl;
  }
  try {
    char prefix = seedRotateEntry.get_text()[0];
    if (prefix == '-') {
      feedAndField.setSeedRotate(-std::stod(seedRotateEntry.get_text().erase(0, 1)));
    } else {
      feedAndField.setSeedRotate(std::stod(seedRotateEntry.get_text()));
    }
  } catch (std::exception e) {
    feedAndField.setSeedRotate(0);
    // std::cout << "Please enter a number of degrees for the seed rotation." << std::endl;
  }
  // std::cout << "ID: " << feedAndField.getSeedID() << std::endl;
  // std::cout << "Seed X: " << feedAndField.getSeedX() << std::endl;
  // std::cout << "Seed Y: " << feedAndField.getSeedY() << std::endl;
  // Reload feed and plotted points
  feedAndField.queue_draw();
  return true;
}

Window::Window() {
  set_title("Fieldspace");
  set_default_size(1538, 550);
  set_child(grid);
  feedAndField.set_hexpand(true);
  feedAndField.set_vexpand(true);
  grid.attach(feedAndField, 0, 0, 1, 1);
  // seedTagEntry.set_max_length(50);
  seedTagEntry.set_text("0");
  seedXEntry.set_text("0");
  seedYEntry.set_text("0");
  seedRotateEntry.set_text("0");
  grid.attach(seedTagEntry, 0, 1, 1, 1);
  grid.attach(seedXEntry, 0, 2, 1, 1);
  grid.attach(seedYEntry, 0, 3, 1, 1);
  grid.attach(seedRotateEntry, 0, 4, 1, 1);

  // seedTagEntry.set_max_length(50);
  // seedTagEntry.set_text(0);

  // Update camera feed at 20 Hz
  Glib::signal_timeout().connect(sigc::mem_fun(*this, &Window::periodic), 50, 0);
}

int main(int argc, char** argv) {
  auto app = Gtk::Application::create();
  return app->make_window_and_run<Window>(argc, argv);
}
