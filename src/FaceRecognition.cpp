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


cv::Rect Face::Dlib(int Arr[])
{
  if (Arr[0] == 1) // only one face
    {
      //printf("X[%d] Y[%d] width[%d] height[%d]\n", Arr[1], Arr[2], Arr[3], Arr[4]);
      Rect roi;
      roi.x = Arr[1];
      roi.y = Arr[2];
      roi.width = Arr[3];
      roi.height = Arr[4];
      return roi;
    }
  else printf("Face detection is wrong!");
}

