//James Rogers Oct 2023 (c) Plymouth University

#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//here
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>  // for cascade classifier
#include <iostream>
#include <vector>
#include <string>


int main()
{
    VideoCapture camera(0);
    CascadeClassifier face_cascade;
    face_cascade.load("C:/AINT308Lib/OpenCV41/release/install/etc/haarcascades/haarcascade_frontalface_default.xml");

    if(face_cascade.empty()){
      cout << "Classifier has not been loaded!\n";
      return 0;
    }

    while(1)
    {
        //capture frame
        Mat frame;
        camera.read(frame);

        //face detect
        vector<Rect> faces;
        face_cascade.detectMultiScale(frame, faces, 1.1,10);

        // drawing bounding box around detected faces
        for(auto& face : faces)
           rectangle(frame, face.tl(), face.br(), Scalar(255, 0, 255), 3);

        //display
        imshow("Face Detection", frame);
        waitKey(10);

        //print pos of first face
        if(faces.size()>=1)
        {
            /*float z = 10000.0f/faces.front().width;
            float x = ((faces.front().x+faces.front().size().width /2-frame.size().width /2)*z)/500.0f;
            float y = ((faces.front().y+faces.front().size().height/2-frame.size().height/2)*z)/500.0f;
            cout<<"("<<x<<", "<<y<<", "<<z<<")"<<endl;*/

            signed int z = 10000.0f/faces.front().width;
            signed int x = ((faces.front().x+faces.front().size().width /2-frame.size().width /2)*z)/500.0f;
            signed int y = -1*((faces.front().y+faces.front().size().height/2-frame.size().height/2)*z)/500.0f;
            cout<<"("<<x<<", "<<y<<", "<<z<<")"<<endl;
        }



    }

}


















