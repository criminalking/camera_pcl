#include "FaceRecognition.h"

Rect Face::Haar(Mat& img)
{
  equalizeHist(img, img);
  CascadeClassifier faces_cascade;
  faces_cascade.load("frontalface.xml");
  vector<Rect> faces;
  faces_cascade.detectMultiScale(img, faces);
  cout << "faces.size(): " << faces.size() << endl;
  if (faces.size() != 0)
    {
      Rect roi = faces[faces.size() - 1];
      rectangle(img, roi, Scalar(0, 0, 255));
      
      char left_name[60];
      char disp_name[60];
      sprintf(left_name, "%d.jpg", face_num);
      ++face_num;
      imwrite(left_name, img);
      
      return roi;
    }
  else
    cout << "Find no face." << endl;
}
