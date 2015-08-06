/* This program is used for tracking faces and displaying images to the head screen. It is also a hub where all of the other main services are called. */

#include "FaceDetector.h"

/* instantiates ros objects and face images */
FaceDetector::FaceDetector() 
{
    xdisplay_pub = n.advertise<sensor_msgs::Image>("/robot/xdisplay", 1000),
    monitor_pub = n.advertise<baxter_core_msgs::HeadPanCommand>("robot/head/command_head_pan", 1000),
    monitor_sub = n.subscribe<baxter_core_msgs::HeadState>("robot/head/head_state", 10, &FaceDetector::updateHead, this),
    raw_image = n.subscribe<sensor_msgs::Image>(/*"camera/rgb/image_raw"*/"/republished/head_camera/image", 1, &FaceDetector::chooseStage, this), 
    
    pickup_client = n.serviceClient<operation_plushie::Pickup>("pickup_service");
    pickup_isComplete_client = n.serviceClient<operation_plushie::isComplete>("pickup_isComplete_service");  
    delivery_client = n.serviceClient<operation_plushie::Deliver>("delivery_service");
    delivery_isComplete_client = n.serviceClient<operation_plushie::isComplete>("delivery_isComplete_service");

    //Check to make sure that the cascade classifier is loaded correctly.
    //The program will fail if it isn't loaded.
    if( !face_cascade.load("../../../src/operation_plushie/res/haarcascade_frontalface_alt.xml") )
    { 
        printf("--(!)Error loading face cascade\n"); 
    }
    
    //Loads the images used for the screen on Baxter's head.
    cv::Mat happy_mat = cv::imread("../../../src/operation_plushie/res/happy.jpg", CV_LOAD_IMAGE_COLOR), 
            unsure_mat = cv::imread("../../../src/operation_plushie/res/unsure.jpg", CV_LOAD_IMAGE_COLOR),
            lemon_mat = cv::imread("../../../src/operation_plushie/res/big_lemongrab.png", CV_LOAD_IMAGE_COLOR);
   
    lemon_face = cv_bridge::CvImage(std_msgs::Header(), "bgr8", lemon_mat).toImageMsg(); 
    happy_face = cv_bridge::CvImage(std_msgs::Header(), "bgr8", happy_mat).toImageMsg();
    unsure_face = cv_bridge::CvImage(std_msgs::Header(), "bgr8", unsure_mat).toImageMsg();
    
    isFirst = true;
    no_face_count = 20;
    state = PICKUP;
}

/* This updates a variable that stores the state of Baxter's head (position, etc.). */
void 
FaceDetector::updateHead(const baxter_core_msgs::HeadState::ConstPtr& msg) {
	head_state = *msg;
}

/* Converts the image into a mat and stores it in a variable, calling detectAndDisplay later on. */
void 
FaceDetector::head_camera_processing(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr_cam;
    try 
    {   
        // This variable is assigned to the output of a function that converts an image to a mat.
        cv_ptr_cam = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }   
    catch (cv_bridge::Exception& e)
    {  
        // Gives an error if it didn't work. 
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    detectAndDisplay(cv_ptr_cam->image);   
}

/* Detects faces and displays a face based on what it sees. */
void 
FaceDetector::detectAndDisplay(cv::Mat frame)
{
    std::vector<cv::Rect> raw_faces;
    cv::Mat frame_gray;

    cv::cvtColor( frame, frame_gray, cv::COLOR_BGR2GRAY );
    cv::equalizeHist( frame_gray, frame_gray );

    //-- Detect faces
    face_cascade.detectMultiScale( frame_gray, raw_faces, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30) );

    std::vector<cv::Rect> confirmed_faces = findConfirmedFaces(raw_faces, frame);
    
    decrementConsistentRects();    

    int best_index = findBestIndex(frame), 
        fromCenter = 1000;

    if(confirmed_faces.size())
        fromCenter = consistent_rects[best_index].rect.x 
                     - consistent_rects[best_index].rect.width/2 - frame.cols/2; 

    tickFaceCount(fromCenter, confirmed_faces.size(), frame); 
    
    if(no_face_count > 0)
        //Display angry face if he hasn't seen a face for any period of time.
        xdisplay_pub.publish(lemon_face);
    else if(abs(fromCenter) > 50)
        //Display unsure face if he sees an uncentered face.
        xdisplay_pub.publish(unsure_face);
    else
        //Display happy face if he sees a centered face.
        xdisplay_pub.publish(happy_face);

    if(!head_state.isPanning && confirmed_faces.size())
    {
        if(abs(fromCenter) > 50)
        {
            baxter_core_msgs::HeadPanCommand msg;
	        msg.speed = 25;
	        msg.target = head_state.pan + (fromCenter > 0 ? -.1 : .1);
    	    monitor_pub.publish(msg);
        } 
        else if(no_face_count <= -1)
        {
            deliver();
        }
    } 
    
    //Displays the camera feed with the face detection overlay on the computer screen.
    cv::imshow("Face Detector", frame);
    cv::waitKey(10);
}

/* Thresholds the face detection image and sees if the face has a certain percentage of the proper color in it. Returns true if the face is properly colored. */
bool 
FaceDetector::properColor(cv::Mat portion)
{
    int iLowH = 0,
        iHighH = 43,
        iLowS = 46,
        iHighS = 106,
        iLowV = 20,
        iHighV = 255;

    cv::Mat imgHSV;

    cv::cvtColor(portion, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    cv::Mat imgThresholded;

    cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);
    //Threshold the image

    //morphological opening (remove small objects from the foreground)
    cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    //morphological closing (fill small holes in the foreground)
    cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    double percentage = (double) cv::countNonZero(imgThresholded) / (imgThresholded.rows*imgThresholded.cols);
   
    return (percentage > .31 && percentage < .7);
}

/* Looks at each face and checks its consistency, assigning a rating to it. */
void 
FaceDetector::addConsistent(cv::Rect r)
{
    // Look through a vector of rectangles that are drawn around each face.
    for(int i = 0; i < consistent_rects.size(); i++)
    {
            // If it's overlapping with a rectangle previously in that area, increase its rating.
            if(isOverlapping(r, consistent_rects[i].rect))
            {
                consistent_rects[i].rect = r;
                consistent_rects[i].rating += 1.1;
                return;
            }
    }
    ConsistentRect newRect;
    newRect.rect = r;
    newRect.rating = 1.1;
    // Push the rectangle into a vector of known, consistent rectangles (faces).
    consistent_rects.push_back(newRect);
}

/* Checks if the rectangle is overlapping using an algorithm. Returns true if this is the case. */
bool 
FaceDetector::isOverlapping(cv::Rect r1, cv::Rect r2)
{
    int overlap_width = std::min(r1.x + r1.width, r2.x + r2.width) > std::max(r1.x, r2.x) ? 1 : 0; 
    int overlap_height = std::min(r1.y, r2.y) > std::max(r1.y - r1.height, r2.y - r2.height) ? 1 : 0;
    return (overlap_width == 1 && overlap_height == 1);
}

/* Decrement the value of the rectangle's rating and erase any rectangle's whose consistencies are too low */
void 
FaceDetector::decrementConsistentRects() {
    for(int i = 0; i < consistent_rects.size(); i++)
    {
        if((consistent_rects[i].rating -= .9) < -1) {
            consistent_rects.erase(consistent_rects.begin() + i);
            i--;
        }
    }
}

/* Finds the best-rated face and draws a red circle around it. */
int 
FaceDetector::findBestIndex(cv::Mat frame) 
{
    int best_index = -1, best_rating = -1;
    cv::Point best_point;

    for(int i = 0; i < consistent_rects.size(); i++)
    {
        if(best_rating < consistent_rects[i].rating)
        {
            best_rating = consistent_rects[i].rating;
            best_index = i;
        }
    }

    if(best_index >= 0) {
        // Sets best_point to be the center of the most consistent rectangle.
        best_point = cv::Point(consistent_rects[best_index].rect.x + consistent_rects[best_index].rect.width/2, consistent_rects[best_index].rect.y + consistent_rects[best_index].rect.height/2);
        
        // Draws a red circle around best_point.
        cv::ellipse( frame, best_point, cv::Size( consistent_rects[best_index].rect.width*0.5, consistent_rects[best_index].rect.height*0.5), 0, 0, 360, cv::Scalar( 0, 0, 255 ), 4, 8, 0 );
    }
    
    return best_index;
}

/* Ticks no_face_count depending on whether there is a face on the screen or not */
/* It counts like this, if it is  */ 
void 
FaceDetector::tickFaceCount(int fromCenter, int confirmed_size, cv::Mat frame) {
    const int FACE_COUNT = 5;    

    //if there is atleast one confirmed face on the screen
    if(confirmed_size)
    {
        if(no_face_count <= -1)
        { 
            if(fromCenter < 50 && fromCenter > -50) 
            {
                no_face_count = -FACE_COUNT;
            } 
            else
            {
                no_face_count = 1;
            }
        }

        if(no_face_count < 0)
            no_face_count = -FACE_COUNT;
        else
            no_face_count--;
    
    } 
    else 
    {
        if(no_face_count > 0)
            no_face_count = FACE_COUNT;
        else
            no_face_count++;
    }   
}

/* Sorts faces that have a proper color and do not have a proper color for further consistent rect filtering 
   draws blue and black circles */
std::vector<cv::Rect> 
FaceDetector::findConfirmedFaces(std::vector<cv::Rect> raw_faces, cv::Mat frame)
{
    std::vector<cv::Rect> confirmed_faces;
    for ( size_t i = 0; i < raw_faces.size(); i++ )
    {
        cv::Point center( raw_faces[i].x + raw_faces[i].width/2, raw_faces[i].y + raw_faces[i].height/2 );

        if(properColor(cv::Mat(frame, raw_faces[i])))
        {
            confirmed_faces.push_back(raw_faces[i]); 
            cv::ellipse( frame, center, cv::Size( raw_faces[i].width*0.5, raw_faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 0 ), 4, 8, 0 );
           
            addConsistent(raw_faces[i]);
        } 
        else
        {
            cv::ellipse( frame, center, cv::Size( raw_faces[i].width*0.5, raw_faces[i].height*0.5), 0, 0, 360, cv::Scalar( 0, 0, 0 ), 4, 8, 0 );
        }
    }

    return confirmed_faces;
}

/* Selects what to do based on its state. */
void 
FaceDetector::chooseStage(const sensor_msgs::ImageConstPtr& msg)
{
    switch(state)
    {
    case PICKUP:
        pickup();
        break;
    case DELIVER:
        head_camera_processing(msg);
        break;
    }

} 

/* Calls the Pickup service and sets the state to Deliver when it's done. */
void 
FaceDetector::pickup()
{
    operation_plushie::Pickup srv;
    srv.request.isFirst = isFirst;
    isFirst = false;

    //Sets stage in pickup to initializing.
    pickup_client.call(srv);
    
    operation_plushie::isComplete pickup_progress;

    // Displays angry face if the arm can't move or a happy face if it can.
    do {
        pickup_isComplete_client.call(pickup_progress);
        if(pickup_progress.response.isStuck)
            xdisplay_pub.publish(lemon_face);
        else
            xdisplay_pub.publish(happy_face);
    } while(!pickup_progress.response.isComplete);

    isLeft = pickup_progress.response.isLeft;
    
    state = DELIVER;
}

/* Calls the Deliver service and changes the state to PICKUP once it's done. */
void 
FaceDetector::deliver()
{
    operation_plushie::Deliver srv;
    srv.request.headPos = head_state.pan;
    srv.request.isLeft = isLeft;   
 
    delivery_client.call(srv);

    operation_plushie::isComplete delivery_progress;

    do {
        delivery_isComplete_client.call(delivery_progress);
    } while(!delivery_progress.response.isComplete);
    
    consistent_rects.clear();
    no_face_count = 0;
    state = PICKUP;
}
