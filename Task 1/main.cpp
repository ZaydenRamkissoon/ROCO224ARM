//James Rogers Oct 2023 (c) Plymouth University

#include <fstream>
#include <sys/types.h>
#include <string>
#include <opencv2/opencv.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>  // for cascade classifier
#include <iostream>
#include <vector>

// serial_posix.cpp
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

#include <windows.h>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

int main()
{
    VideoCapture camera(0);
    CascadeClassifier face_cascade;
    face_cascade.load("C:/AINT308Lib/OpenCV41/release/install/etc/haarcascades/haarcascade_frontalface_default.xml");

    if(face_cascade.empty()){
      cout << "Classifier has not been loaded!\n";
      return 0;
    }

    std::string portName = "\\\\.\\COM8";  // COM in Windows format
    DWORD baudRate = 9600;

    // Open the serial port
    HANDLE serialHandle = CreateFileA(
        portName.c_str(),                  // Port name
        GENERIC_READ | GENERIC_WRITE,      // Read/Write access
        0,                                 // No sharing
        NULL,                              // Default security attributes
        OPEN_EXISTING,                     // Open existing port only
        0,                                 // Non-overlapped I/O
        NULL                               // Null template
        );

    // Check if port opened successfully
    if (serialHandle == INVALID_HANDLE_VALUE) {
        DWORD error = GetLastError();
        std::cerr << "Error opening serial port COM. Error code: " << error << std::endl;
        return 1;
    }

    std::cout << "Serial port COM opened successfully." << std::endl;

    // Configure the port settings
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    // Get the current configuration
    if (!GetCommState(serialHandle, &dcbSerialParams)) {
        std::cerr << "Error getting serial port state." << std::endl;
        CloseHandle(serialHandle);
        return 1;
    }

    // Set the parameters
    dcbSerialParams.BaudRate = baudRate;      // Set baud rate to 9600
    dcbSerialParams.ByteSize = 8;             // 8 data bits
    dcbSerialParams.StopBits = ONESTOPBIT;    // 1 stop bit
    dcbSerialParams.Parity = NOPARITY;        // No parity

    // Apply the new configuration
    if (!SetCommState(serialHandle, &dcbSerialParams)) {
        std::cerr << "Error setting serial port state." << std::endl;
        CloseHandle(serialHandle);
        return 1;
    }

    // Set timeouts
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(serialHandle, &timeouts)) {
        std::cerr << "Error setting timeouts." << std::endl;
        CloseHandle(serialHandle);
        return 1;
    }

    // Open the camera and begin face detection
    while(1)
    {
        // Capture frame
        Mat frame;
        camera.read(frame);

        // Face detection
        vector<Rect> faces;
        face_cascade.detectMultiScale(frame, faces, 1.1, 10);

        // Drawing bounding box around detected faces
        for(auto& face : faces)
           rectangle(frame, face.tl(), face.br(), Scalar(255, 0, 255), 3);

        // Display the frame
        imshow("Face Detection", frame);
        waitKey(10);

        // Print position of the first face
        if(faces.size() >= 1)
        {
            signed int z = 10000.0f / faces.front().width;
            signed int x = ((faces.front().x + faces.front().size().width / 2 - frame.size().width / 2) * z) / 500.0f;
            signed int y = -1 * ((faces.front().y + faces.front().size().height / 2 - frame.size().height / 2) * z) / 500.0f;

            cout << "(" << x << ", " << y << ", " << z << ")" << endl;

            // Prepare the message to send
            char buffer[16];
            sprintf(buffer, "(%3d,%3d,%3d)", x, y, z);

            // Send the coordinates over the serial port
            DWORD bytesWritten;
            bool success = WriteFile(
                serialHandle,               // Handle to the serial port
                buffer,                     // Data to send (buffer)
                strlen(buffer),             // Length of data (strlen(buffer) ensures the null terminator is not included)
                &bytesWritten,              // Number of bytes written
                NULL                        // Not using overlapped I/O
            );

            // Check if write was successful
            if (!success) {
                std::cerr << "Error writing to serial port." << std::endl;
                CloseHandle(serialHandle);
                return 1;
            }

            std::cout << "Sent coordinates: " << buffer << std::endl;
            std::cout << "Bytes written: " << bytesWritten << std::endl;
        }
    }

    // Close the serial port
    CloseHandle(serialHandle);
    std::cout << "Serial port closed." << std::endl;
}










