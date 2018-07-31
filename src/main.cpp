
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include "opencv2/imgproc.hpp"
//#include "opencv2/highgui.hpp"
#include <iostream>
#include "cannylive/errorDistances.h"
//using namespace cv;


cv::Mat src_gray;
cv::Mat dst, detected_edges;
cv::Mat canny_output;

int lowThreshold = 45;
const int max_lowThreshold = 100;
const int ratio = 3;
const int kernel_size = 3;

ros::Publisher* pub; // global variable for publisher so we can send inside CBfunc

int middlex;
int middley;
int left;
int right;
int middle;

void CBfunc(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;


	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	//
	cvtColor( cv_ptr->image, src_gray, cv::COLOR_BGR2GRAY);
	dst.create( cv_ptr->image.size(), cv_ptr->image.type()); // creates dst, same size and type as cv_ptr->src
  cv::blur( src_gray, canny_output, cv::Size(3,3));
    //cv::GaussianBlur( src_gray, canny_output, cv::Size(3,3), 0); // alternative blurring method
  int cannythres = 35;
  cv::Canny( canny_output, canny_output, cannythres, cannythres*ratio, kernel_size );

    //cv::imshow("cannyraw", canny_output);
  dst = cv::Scalar::all(0); //sets all values to 0?

    //imshow("detected_edges", detected_edges );
 
    //src.copyTo( dst, detected_edges);
    src_gray.copyTo( dst, canny_output);

  
    middlex = round(dst.size().width*0.5);
    middley = round(dst.size().height*0.5);
    left = 0;
    right = 0;
    middle = middlex;

       
    for(int j = 479; j > 0; j--)   // hiddencropped is 1280 x 470
    { 
    	if(dst.at<uchar>(j,middle) == 0)
    	{

		  for (int i=middle;i<dst.cols;i++)
		  {  
		    if( dst.at<uchar>(j,i) == 0 && i != dst.cols-1) // !!crashes here!!
		    {   
		         //dst.at<uchar>(j,i) = 255; //white
		        
		    }
		    else
		    {

		      right = i;
		      break;
		    }           
		       
		  }

		  for (int i=middle-1;i>0;i--)
		  {
		    if( dst.at<uchar>(j,i) == 0 && i != dst.cols-1)
		    {
		      //dst.at<uchar>(j,i) = 255; //white
		    }
		    else
		    {
		      left = i;
		      break;
		    }  
	      }
	            middle = round(right + (left - right)*0.5);
  				dst.at<uchar>(j,middle) = 255;         
           
        }
        else
        {
        	break;
        }
      


    }
   
      cannylive::errorDistances emsg;
       
      int k;
      /*
      for(k=0; k<10; k++)
      {
        emsg.errorDist[k] = k;
      }

      */
      emsg.errorDist.push_back(0);
      emsg.errorDist.push_back(1);
      emsg.errorDist.push_back(2);
      emsg.errorDist.push_back(3);

      //ROS_INFO("test\n");

      pub->publish(emsg);
    //aaa
    //cv::imshow( "test", dst );
    cv::waitKey(1);
    //ROS_INFO("test\n");
}


    /*# =============================================================================
#           Read fotage from video buffer
# =============================================================================
            returnCode, resolution, image = vrep.simxGetVisionSensorImage(clientID, camera, 0, vrep.simx_opmode_buffer)
            if returnCode == vrep.simx_return_ok:
# =============================================================================
#               Store the resolution in a column and row variable
# =============================================================================
                cols = resolution[0]
                rows = resolution[1]
# =============================================================================
#               Convert image to a suitable format for openCV
# =============================================================================
                imageByteArray = bytes(array.array('b', image))
                imageBuffer = I.frombuffer("RGB", (cols,rows ), imageByteArray, "raw", "RGB", 0, 1)
                img2 = np.asarray(imageBuffer)
# =============================================================================
#               Convert image to grayscale
# =============================================================================
                img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
# =============================================================================
#               Smooth the image with Gaussian blur
# =============================================================================
                img2 = cv2.blur(img2, (3,3))
# =============================================================================
#               Do edgedetection with canny
# =============================================================================
                img2 = cv2.Canny(img2, 45, 100)
# =============================================================================
#               Algorithm for drawing a virtual line
# =============================================================================
                center = int(cols/2)
                left = 0
                right = 0
                middle = center;
                mean = 0
                minPx = 20
                maxPx = 50
                for jdx in range(minPx, maxPx+1):
                    if img2[jdx, middle] == 0:
                        for idx in range(middle, cols, 1):
                            if img2[jdx, idx] == 255 or idx == cols-1:
                                right = idx
                                break
                        for idx in range(middle-1, -1, -1):
                            if img2[jdx, idx] == 255 or idx == 0:
                                left = idx
                                break
                    middle = int(round(right+(left-right)*0.5))
                    mean += middle
                    img2[jdx, middle] = 255
                    img2[jdx, center] = 128
# =============================================================================
#               Calculate the error between the middle of the road and the centerline of the car
# =============================================================================
                middle = mean/(maxPx-minPx)
                error = center-middle
                for idx in range(minPx, maxPx+1):
                        img2[idx, int(middle)] = 190
                plt.imshow(img2, cmap = "viridis", origin = "lower")
                plt.show()
            elif returnCode == vrep.simx_return_novalue_flag:
                print('No data yet')
                pass
            else:
                sys.exit('Could not get image')

### end
*/




int main(int argc, char* argv[])
{
	ros::init(argc, argv, "cannylive");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub;

	//image_transport::Publisher publisher = it.advertise("camera/canny", 1);
    ros::Publisher publisher = nh.advertise<cannylive::errorDistances>("errorDistances", 1); //
	pub = &publisher; // point global ptr to the publisher declared above.
	//image_sub = it.subscribe("/camera/color/image_raw", 1, CBfunc);
	image_sub = it.subscribe("/videofile/image_raw", 1, CBfunc);

	ros::spin();

	return 0;

}