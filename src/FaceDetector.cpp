#include "FaceDetector.h"

FaceDetector::FaceDetector() 
{
    xdisplay_pub = n.advertise<sensor_msgs::Image>("/robot/xdisplay", 1000),
    monitor_pub = n.advertise<baxter_core_msgs::HeadPanCommand>("robot/head/command_head_pan", 1000),
    monitor_sub = n.subscribe<baxter_core_msgs::HeadState>("robot/head/head_state", 10, &FaceDetector::updateHead, this),
    raw_image = n.subscribe<sensor_msgs::Image>(/*"camera/rgb/image_raw"*/"/cameras/head_camera/image", 10, &FaceDetector::call_back, this), 
    delivery_client = n.serviceClient<image_reader::Wave>("delivery_server");
   
    if( !face_cascade.load("haarcascade_frontalface_alt.xml") )
    { 
        printf("--(!)Error loading face cascade\n"); 
    }
    
    cv::Mat happy_mat = cv::imread("happy.jpeg", CV_LOAD_IMAGE_COLOR), 
            unsure_mat = cv::imread("unsure.jpeg", CV_LOAD_IMAGE_COLOR);
    
    happy_face = cv_bridge::CvImage(std_msgs::Header(), "bgr8", happy_mat).toImageMsg();
    unsure_face = cv_bridge::CvImage(std_msgs::Header(), "bgr8", unsure_mat).toImageMsg();
    no_face_count = 20;
    isMoving = false;
}

void FaceDetector::begin_detection()
{
    ros::spin();
}

void FaceDetector::updateHead(const baxter_core_msgs::HeadState::ConstPtr& msg) {
	head_state = *msg;
}

void FaceDetector::call_back(const sensor_msgs::ImageConstPtr& msg)
{
    if(isMoving)
        return;

    cv_bridge::CvImagePtr cv_ptr_cam;
    try 
    {   
        cv_ptr_cam = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }   
    catch (cv_bridge::Exception& e)
    {   
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    detectAndDisplay(cv_ptr_cam->image);   
}

void FaceDetector::detectAndDisplay(cv::Mat frame)
{
    std::vector<cv::Rect> raw_faces;
    cv::Mat frame_gray;

    cv::cvtColor( frame, frame_gray, cv::COLOR_BGR2GRAY );
    cv::equalizeHist( frame_gray, frame_gray );

    //-- Detect faces
    face_cascade.detectMultiScale( frame_gray, raw_faces, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30) );

    std::vector<cv::Rect> confirmed_faces = findConfirmedFaces(raw_faces, frame);
    
    decrementConsistentRects();    

    int best_index = findBestIndex(frame);

    tickFaceCount(best_index, confirmed_faces.size(), frame); 
 
    if(!head_state.isPanning && confirmed_faces.size() && !isMoving)
    {
        int fromCenter = consistent_rects[best_index].rect.x - consistent_rects[best_index].rect.width/2 - frame.cols/2; 
        if(fromCenter > 50 || fromCenter < -50)
        {
            baxter_core_msgs::HeadPanCommand msg;
	        msg.speed = 25;
	        msg.target = head_state.pan + (fromCenter > 0 ? -.1 : .1);
    	    monitor_pub.publish(msg);
        }
    } 

    if(no_face_count == 1)
        xdisplay_pub.publish(unsure_face);
    else if(no_face_count == -1) {
        xdisplay_pub.publish(happy_face);
    }

    cv::imshow("Test", frame);
    cv::waitKey(10);
}

bool FaceDetector::properColor(cv::Mat portion)
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
   
    return (percentage > .35);
}

void FaceDetector::addConsistent(cv::Rect r)
{
    for(int i = 0; i < consistent_rects.size(); i++)
    {
            if(isOverlapping(r, consistent_rects[i].rect))
            {
                ROS_INFO("Overlap found");
                consistent_rects[i].rect = r;
                consistent_rects[i].rating += 1.1;
                return;
            }
    }
    ROS_INFO("Overlap not found");
    ConsistentRect newRect;
    newRect.rect = r;
    newRect.rating = 1.1;
    consistent_rects.push_back(newRect);
}

bool FaceDetector::isOverlapping(cv::Rect r1, cv::Rect r2)
{
    int overlap_width = std::min(r1.x + r1.width, r2.x + r2.width) > std::max(r1.x, r2.x) ? 1 : 0; 
    int overlap_height = std::min(r1.y, r2.y) > std::max(r1.y - r1.height, r2.y - r2.height) ? 1 : 0;
    return (overlap_width == 1 && overlap_height == 1);
}
void FaceDetector::decrementConsistentRects() {
    for(int i = 0; i < consistent_rects.size(); i++)
    {
        if((consistent_rects[i].rating -= .9) < -1) {
            consistent_rects.erase(consistent_rects.begin() + i);
            i--;
        }
    }
}

int FaceDetector::findBestIndex(cv::Mat frame) 
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
        best_point = cv::Point(consistent_rects[best_index].rect.x + consistent_rects[best_index].rect.width/2, consistent_rects[best_index].rect.y + consistent_rects[best_index].rect.height/2);
        
        cv::ellipse( frame, best_point, cv::Size( consistent_rects[best_index].rect.width*0.5, consistent_rects[best_index].rect.height*0.5), 0, 0, 360, cv::Scalar( 0, 0, 255 ), 4, 8, 0 );
    }
    
    return best_index;
}

void FaceDetector::tickFaceCount(int best_index, int confirmed_size, cv::Mat frame) {
    if(confirmed_size)
    {
        int fromCenter 
            = consistent_rects[best_index].rect.x - consistent_rects[best_index].rect.width/2 - frame.cols/2;
        
        if(no_face_count == -1)
        { 
            if(fromCenter < 50 && fromCenter > -50) 
            {
                isMoving = true;
                no_face_count = -20;
                
                image_reader::Wave srv;
                srv.request.headPos = head_state.pan;
                delivery_client.call(srv);

                isMoving = false;
            } 
            else
            {
                no_face_count = 1;
            }
        }

        if(no_face_count < 0)
            no_face_count = -20;
        else
            no_face_count--;
    
    } 
    else 
    {
        if(no_face_count > 0)
            no_face_count = 20;
        else
            no_face_count++;
    }   
}

std::vector<cv::Rect> FaceDetector::findConfirmedFaces(std::vector<cv::Rect> raw_faces, cv::Mat frame)
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
