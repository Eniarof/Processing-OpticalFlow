// OpticalFlow
// The Book of Eniarof
// code: Douglas Edric Stanley
// with contribution from Hidetoshi Shimodaira
// GPL v3.0

// import Processing video library
import processing.video.*;
// video capture object
Capture video;

void setup() {
  size(640, 480);
  // setup optical flow
  setupFlow(width, height);
}

void draw() {
  // update Optical Flow
  updateFlow();
  // draw camera image
   image(video, 0, 0);
   // draw flow grid
   drawFlow();
   // draw mean arrow
   drawMean();
}