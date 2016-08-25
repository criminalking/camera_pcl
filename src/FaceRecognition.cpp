#include "FaceRecognition.h"

cv::Rect Face::Haar(Mat& img)
{
  equalizeHist(img, img);
  CascadeClassifier faces_cascade;
  faces_cascade.load("frontalface.xml");
  std::vector<cv::Rect> faces;
  faces_cascade.detectMultiScale(img, faces, 1.1, 3, 0|CV_HAAR_SCALE_IMAGE, Size(34, 34)); // 1.1 ScaleFactor, 3 minNeighbours
  cout << "faces.size(): " << faces.size() << endl;
  if (faces.size() != 0)
    {
      cv::Rect roi = faces[0];
      cv::rectangle(img, roi, Scalar(0, 0, 255));
      
      char left_name[60];
      sprintf(left_name, "%d.jpg", face_num);
      ++face_num;
      imwrite(left_name, img);
      
      return roi;
    }
  else
    cout << "Find no face." << endl;
}


cv::Rect Face::Dlib(Mat& img_opencv)
{
  // frontal_face_detector detector = get_frontal_face_detector();
  // image_window win;

  // cv_image<unsigned char> img(img_opencv);
  // //array2d<unsigned char> img(img_opencv);
  // pyramid_up(img);

  // // Now tell the face detector to give us a list of bounding boxes around all the faces it can find in the image.
  // std::vector<dlib::rectangle> dets = detector(img);
  // cout << "Number of faces detected: " << dets.size() << endl;
  // Rect roi;
  // roi.x = dets[0].left();
  // roi.y = dets[0].top();
  // roi.width = dets[0].right() - dets[0].left();
  // roi.height = dets[0].bottom() - dets[0].top();
  
  // // Now we show the image on the screen and the face detections as red overlay boxes.
  // //win.clear_overlay();
  // //win.set_image(img);
  // //win.add_overlay(dets, rgb_pixel(255,0,0));
  
  // return roi;
}
