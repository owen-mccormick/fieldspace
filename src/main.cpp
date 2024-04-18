#include <iostream>
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

extern "C" {
  #include "apriltag.h"
  #include "tag36h11.h"
  #include "common/getopt.h"
}

// using json = nlohmann::json;

// Are top-level statements like this a good idea?
// Should be relocated inside of the Window class
cv::VideoCapture cap(0);
apriltag_family_t* tf = tag36h11_create(); // 36h11 tag family used in FRC
apriltag_detector_t* td = apriltag_detector_create();

struct Tag {
  int id;
  double pitch, yaw, roll, i, j, k;
  bool isSeed;
  bool isGauged;
};

// Subclass of entry box that only accepts numeric input (https://stackoverflow.com/questions/10279579/force-numeric-input-in-a-gtkentry-widget#10282948)
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
  for(int i = 0; i < text.length(); i++) {
    if(Glib::Unicode::isdigit(text[i]) == false) return false;
  }
  return true;
}

// FeedAndField: Represents an OpenCV camera feed and field map drawn in a gtkmm DrawingArea
class FeedAndField : public Gtk::DrawingArea {
  public:
    FeedAndField();
    virtual ~FeedAndField();
  protected:
    void on_draw(const Cairo::RefPtr<Cairo::Context>& cr, int width, int height);
    // void FeedAndField::draw_text(const Cairo::RefPtr<Cairo::Context>& cr, int x, int y, int rectangle_width, int rectangle_height, std::string text);
    // apriltag_family_t* tf;
    // apriltag_detector_t* td;
    const int fieldX = 485;
    const int fieldY = 2;
};

FeedAndField::FeedAndField() {
  apriltag_detector_add_family(td, tf);
  // TODO - tune
  td->quad_decimate = 2.0;
  td->quad_sigma = 0; // blur
  td->nthreads = 1;
  td->debug = 0;
  td->refine_edges = 1;
  set_draw_func(sigc::mem_fun(*this, &FeedAndField::on_draw));
}

FeedAndField::~FeedAndField() {
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
}

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

void FeedAndField::on_draw(const Cairo::RefPtr<Cairo::Context>& cr, int width, int height) {
  cv::Mat frame, gray;
  cap >> frame;
  // frame = cv::imread("/home/omccormick/dev/fieldspace/resources/tagformats_web.png");
  cvtColor(frame, gray, cv::COLOR_BGR2GRAY); // black and white image

  // AprilTag detections
  // Make an image_u8_t header for the Mat data
  image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};

  zarray_t* detections = apriltag_detector_detect(td, &im); // TODO - this line causes segfaults

  // Draw detection outlines
  // Largely based on example at https://github.com/AprilRobotics/apriltag/blob/master/example/opencv_demo.cc
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

  // Draw dots on the map at tag positions (test)

  for (double i = 0; i <= 1; i += 0.1) {
    cr->save();
    cr->arc(fieldX + i * field->get_width(), fieldY + i * field->get_height(), 10, 0, 2.0 * M_PI);
    cr->set_source_rgb(0, 0.8, 0);
    cr->fill_preserve();
    cr->restore();
    cr->stroke();
  }
}

class Window : public Gtk::Window {
  public:
    Window();
  protected:
    Gtk::Grid grid;
    FeedAndField feedAndField;
    NumericEntry seedTagEntry, seedXEntry, seedYEntry;
    bool periodic();
};

bool Window::periodic() {
  feedAndField.queue_draw();
  // std::cout << "Redraw..." << std::endl;
  return true;
}

Window::Window() {
  set_title("Fieldspace");
  set_default_size(1538, 530);
  set_child(grid);
  feedAndField.set_hexpand(true);
  feedAndField.set_vexpand(true);
  grid.attach(feedAndField, 0, 0, 1, 1);
  // seedTagEntry.set_max_length(50);
  // seedTagEntry.set_text("0000");
  // seedXEntry.set_text("Seed X...");
  // seedYEntry.set_text("Seed Y...");
  grid.attach(seedTagEntry, 0, 1, 1, 1);
  grid.attach(seedXEntry, 0, 2, 1, 1);
  grid.attach(seedYEntry, 0, 3, 1, 1);

  // seedTagEntry.set_max_length(50);
  // seedTagEntry.set_text(0);

  // Update camera feed at 20 Hz
  Glib::signal_timeout().connect(sigc::mem_fun(*this, &Window::periodic), 50, 0);
}

int main(int argc, char** argv) {
  auto app = Gtk::Application::create();
  return app->make_window_and_run<Window>(argc, argv);
}
