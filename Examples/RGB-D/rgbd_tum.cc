/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <sys/un.h>
#include <stdlib.h>

#include<unistd.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include <Eigen/Core>

#include "Geometry.h"
#include <System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void LoadBoundingBoxFromPython(const string& resultFromPython, std::pair<vector<double>, int>& detect_result);
void MakeDetect_result(vector<std::pair<vector<double>, int>>& detect_result, int sockfd);
cv::Mat GetDynamicBox(vector<std::pair<vector<double>, int>>& detect_result , int sockfd, const cv::Mat &depthmap);
void ShowMask(string mask_name,cv::Mat &mask)
{
    for (int i = 0; i < mask.rows; i++) {
        for (int j = 0; j < mask.cols; j++) {
            // 访问像素值
            int pixel_value = mask.at<uchar>(i, j);
            // 在此处进行处理操作
            if (pixel_value == 0 )
            {
                mask.at<uchar>(i, j) = 255;
            }
        }
    }
    cv::imshow(mask_name, mask);
    for (int i = 0; i < mask.rows; i++) {
        for (int j = 0; j < mask.cols; j++) {
            // 访问像素值
            int pixel_value = mask.at<uchar>(i, j);
            // 在此处进行处理操作
            if (pixel_value == 255 )
            {
                mask.at<uchar>(i, j) = 0;
            }
        }
    }
}


int main(int argc, char **argv)
{
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    //for yolov5
    int sockfd;
	int len;
	struct sockaddr_un address;
	int result;
 
	if((sockfd = socket(AF_UNIX, SOCK_STREAM, 0))==-1)//创建socket，指定通信协议为AF_UNIX,数据方式SOCK_STREAM
	{
		perror("socket");
		exit(EXIT_FAILURE);
	}
	
	//配置server_address
	address.sun_family = AF_UNIX;
	strcpy(address.sun_path, "/home/yakai/SLAM/orbslam_addsemantic-main/yolov5_RemoveDynamic/detect_speedup_send");
	len = sizeof(address);
 
	result = connect(sockfd, (struct sockaddr *)&address, len);
 
	if(result == -1) 
	{
		printf("ensure the server is up\n");
        	perror("connect");
        	exit(EXIT_FAILURE);
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD, depthmap;
    vector<std::pair<vector<double>, int>> detect_result;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        depthmap = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],cv::IMREAD_GRAYSCALE);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        cout << "********new********** " << ni+1 << endl;
        cv::Mat mask = cv::Mat::ones(480,640,CV_8U);
        //mask.setTo(1);
        mask = GetDynamicBox(detect_result,sockfd,depthmap);
        ShowMask("mask yolov5", mask);
        //cv::imshow("depthmap", depthmap);
        //cv::waitKey(0);
        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,depthmap,mask,tframe,detect_result);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double total_time = std::chrono::duration_cast<std::chrono::duration<double> >(start - end).count();
    std::cout << total_time << std::endl;

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
//在一句话中提取出四个边框值和物体类别,such as: left:1 top:134 right:269 bottom:478 class:person 0.79
void LoadBoundingBoxFromPython(const string& resultFromPython, std::pair<vector<double>, int>& detect_result){
    
    if(resultFromPython.empty())
    {
        cerr << "no string from python! " << endl;
    }
    // cout << "here is LoadBoundingBoxFromPython " << endl;
    vector<double> result_parameter;
    int sum = 0, num_bit = 0;

    for (char c : resultFromPython) {//读取数字.    例如读取"748",先读7,再7*10+8=78,再78*10+4,最后读到空格结束
        if (c >= '0' && c <= '9') {
            num_bit = c - '0';
            sum = sum * 10 + num_bit;
        } else if (c == ' ') {
            result_parameter.push_back(sum);
            sum = 0;
            num_bit = 0;
        }
    }

    detect_result.first = result_parameter;
    // cout << "detect_result.first size is : " << detect_result.first.size() << endl;

    string idx_begin = "class:";//读取物体类别
    int idx = resultFromPython.find(idx_begin);
    string idx_end = "0.";
    int idx2 = resultFromPython.find(idx_end);
    string class_label;
    for (int j = idx + 6; j < idx2-1; ++j){
        class_label += resultFromPython[j];
    }

    int class_id = -1;//存入识别物体的种类

    if (class_label == "tv" ||   //低动态物体(在程序中可以假设为一直静态的物体):tv,refrigerator
        class_label == "refrigerator" || 
        class_label == "teddy bear"||
        class_label == "laptop") {
        class_id = 1;
    }

    if (class_label == "chair" || //中动态物体,在程序中不做先验动态静态判断
        class_label == "car"){
        class_id =2;
    } 

    if (class_label == "person") { //高动态物体:人,动物等
        class_id = 3;
    }

    detect_result.second = class_id;
    // cout << "LoadBoundingBoxFromPython class id is: " << class_id << endl;

}

//通过UNIX的协议,从python进程中获取一帧图像的物体框
void MakeDetect_result(vector<std::pair<vector<double>, int>>& detect_result , int sockfd){
    detect_result.clear();

	std::pair<vector<double>, int> detect_result_str;
    int byte;
	char send_buf[50],ch_recv[1024];

    memset(send_buf, 0, sizeof(send_buf)); // clear char[]
    memset(ch_recv, 0, sizeof(ch_recv));   // clear char[]

    sprintf(send_buf, "ok"); // 用sprintf事先把消息写到send_buf
    if ((byte = write(sockfd, send_buf, sizeof(send_buf))) == -1)
    {
        perror("write");
        exit(EXIT_FAILURE);
    }

    if ((byte = read(sockfd, &ch_recv, 1024)) == -1)
    {
        perror("read");
        exit(EXIT_FAILURE);
    }
    //printf("In C++: %s",ch_recv);
    // cout << ch_recv
    //cout << "********new*********" << endl;
    // string ch_recv_string = ch_recv;
    // cout << ch_recv_string << endl;

    char *ptr;//char[]可读可写,可以修改字符串的内容。char*可读不可写，写入就会导致段错误
    ptr = strtok(ch_recv, "*");//字符串分割函数
    while(ptr != NULL){
        printf("ptr=%s\n",ptr);
        string ptr_str = ptr;
        LoadBoundingBoxFromPython(ptr_str,detect_result_str);
        
        detect_result.emplace_back(detect_result_str);
        // cout << "hh: " << ptr_str << endl;  
        ptr = strtok(NULL, "*");
    }
    // cout << "detect_result size is : " << detect_result.size() << endl;
    // for (int k=0; k<detect_result.size(); ++k)
        // cout << "detect_result is : \n " << detect_result[k].second << endl;
}
cv::Mat GetDynamicBox(vector<std::pair<vector<double>, int>>& detect_result , int sockfd, const cv::Mat &depthmap){
    MakeDetect_result(detect_result,sockfd);
    cv::Mat mask = cv::Mat::ones(480,640,CV_8U);
    mask.setTo(1);

    cv::Mat Depth = depthmap.clone();
    for(int k=0; k<detect_result.size(); ++k){
        if (detect_result[k].second == 3 ){
            cv::Point pt11,pt22;
            pt11 = cv::Point(detect_result[k].first[0],detect_result[k].first[1]);
            pt22 = cv::Point(detect_result[k].first[2],detect_result[k].first[3]);
            cv::Rect roi(pt11.x, pt11.y, pt22.x-pt11.x, pt22.y-pt11.y);
            cv::Mat roiImg = Depth(roi);
            int roiImg_size = roiImg.rows*roiImg.cols;
            cout << "roiImg区域大小="<< roiImg_size << endl;
            if(roiImg_size < mask.rows*mask.cols/50)//检测框比较小，则将检测框全部设为mask
            {
                cv::Mat _mask = cv::Mat::ones(480,640,CV_8U);
                _mask.setTo(1);
                roiImg.setTo(0);
                cv::Mat roiDest = _mask(roi);
                roiImg.copyTo(roiDest); 
                mask = mask & _mask;
            }
            else
            {
                //cv::imshow("roiImg1", roiImg);
                for (int i = 0; i < roiImg.rows; i++) {                                                    
                    for (int j = 0; j < roiImg.cols; j++) {
                        int pixel_value = roiImg.at<uchar>(i, j);
                        if (pixel_value < 10)
                        {
                            roiImg.at<uchar>(i, j) = 255;
                        }
                    }
                }
                //cv::imshow("roiImg2", roiImg);


                int histSize = 256;
                float range[] = { 0, 256 };
                const float* histRange[] = { range };
                cv::Mat hist;
                cv::calcHist(&roiImg, 1, 0, cv::Mat(), hist, 1, &histSize, histRange);

                bool find_Threshold = false;
                int threshold = -1;
                for (int i = 1; i < hist.rows; i++)
                {
                    if(find_Threshold == false && hist.at<float>(i) > 1000)
                    {
                        find_Threshold = true;
                        threshold = i + 10;
                    }
                }
                cout << "threshold前后15个的灰度值 "<< endl;
                for (int i = threshold -15; i < threshold + 15; i++)
                {
                    if(i == threshold)
                        cout << "灰度值 " << i << " 的个数：" << hist.at<float>(i) << "    这个是阈值"<< endl;
                    else
                        cout << "灰度值 " << i << " 的个数：" << hist.at<float>(i) << endl;
                }
                cv::threshold(roiImg, roiImg, threshold, 1, cv::THRESH_BINARY_INV);
                cv::Mat depth_roi = roiImg.clone();
                //ShowMask("roiImg3", depth_roi);

                int dilation_size = 7;
                cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                                    cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                    cv::Point( dilation_size, dilation_size ) );
                depth_roi.cv::Mat::convertTo(depth_roi,CV_8U);
                cv::dilate(depth_roi, depth_roi, kernel);
                //ShowMask("roiImg4", depth_roi);
                //cv::imwrite("/home/yakai/SLAM/my_orb2/person.jpg", depth_roi);
                //cv::waitKey(0);

                cv::Mat _depth_roi = cv::Mat::ones(depth_roi.rows,depth_roi.cols,CV_8U);
                depth_roi = _depth_roi - depth_roi;
                cv::Mat _mask = cv::Mat::ones(480,640,CV_8U);
                _mask.setTo(1);
                cv::Mat roiDest = _mask(roi);
                depth_roi.copyTo(roiDest); 
                mask = mask & _mask;
            }
        }
    }
    return mask;
}


