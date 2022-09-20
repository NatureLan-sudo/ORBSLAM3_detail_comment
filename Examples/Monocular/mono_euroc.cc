/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

/**
 * @description:  根据输入量加载图像
 * @param {string} &strImagePath 图像路径
 * @param {string} &strPathTimes 时间戳路径
 * @param {vector<string>} &vstrImages 图像名的string形式容器
 * @param {vector<double>} &vTimeStamps 时间戳string类型容器
 * @return {*}
 */
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{   // * step 0 检查输入量 不小于5
    // 1.词典路径 2.配置路径 3.图像序列地址 4.时间戳地址 （5.轨迹文件）可选的
    // 区别于ORBSLAM2，多了轨迹文件的可选项，支持输出轨迹文件？
    // 支持输入多个图像序列和对应的时间戳文件
    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_euroc path_to_vocabulary path_to_settings  path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << endl;
        return 1;
    }

    const int num_seq = (argc-3)/2; // 确定输入序列的个数
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }
    //* step 1 判断输入的序列的个数，并加载所有输入的序列
    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageFilenames;  
    vector< vector<double> > vTimestampsCam;
    vector<int> nImages;

    vstrImageFilenames.resize(num_seq); //大小为输入序列的个数
    vTimestampsCam.resize(num_seq);     //与时间戳大小一致
    nImages.resize(num_seq);            //与序列数一致，记录每个序列的图像的个数

    int tot_images = 0;                 //图像的总数
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";
    // 通过简单计算，第一输入图像地址，第二输入时间戳地址，存储在以sep为标号的序列中
        LoadImages(string(argv[(2*seq)+3]) + "/mav0/cam0/data", string(argv[(2*seq)+4]), vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;
        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
    }

    // Vector for tracking time statistics
    // 用于记录跟踪时间的容器
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);     //大小是所有图像的总数，应该是要记录下处理完每帧图像的时间

    cout << endl << "-------" << endl;
    cout.precision(17);


    int fps = 20;         //应该是一个频率，但是不知道这个频率是干啥的
    float dT = 1.f/fps;

    //* step 3 加载SLAM系统
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(
        argv[1],                              // path_to_vocabulary 词典路径
        argv[2],                              // path_to_settings   配置路径 mono_euroc
        ORB_SLAM3::System::MONOCULAR,         // 单目模式
        false);                               // 不启用可视化查看器
    float imageScale = SLAM.GetImageScale();  // 获取图像尺度？

    double t_resize = 0.f;
    double t_track = 0.f;
    //* step 4 依次追踪每个序列中的每一张图像
    for (seq = 0; seq<num_seq; seq++) // 由于多序列，相比下多加一个循环，依次加载序列
    {

        // Main loop
        cv::Mat im;
        int proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {

            // Read image from file
            // step 4.1 读根据前面获得的图像文件名读取图像,读取过程中不改变图像的格式 
            im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED); //,CV_LOAD_IMAGE_UNCHANGED);
            double tframe = vTimestampsCam[seq][ni]; //时间戳
            // step 4.2 图像的合法性检查 
            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
                return 1;
            }
            // 根据尺度要求对图像进行处理， 图像处理有计时
            if(imageScale != 1.f) // 如果图像有尺度要求，那么就要对图像的尺寸进行处理
            {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
                int width = im.cols * imageScale;       // 改变图像列向长度
                int height = im.rows * imageScale;      // 改变图像横向长度
                cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
                t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                SLAM.InsertResizeTime(t_resize);
#endif
            }
            // step 4.3 开始计时
    //#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //#else
            //std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now(); 
            // 这里是为了适应不同版本编译的。如果是CXX0X例如c++03就会调用这个， 由于我们会使用c++11，避免编译器划红线，我就注释掉了
    //#endif

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
            //im 当前帧图像， tframe 当前帧图像的时间戳
            // step 4.4 追踪当前图像
            SLAM.TrackMonocular(im,tframe); // TODO change to monocular_inertial
            // step 4.5 追踪完成，停止当前帧的图像计时，并计算追踪耗时
    //#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    //#else
           // std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    //#endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            // step 4.6 根据图像时间戳中记录的两张图像之间的时间和现在追踪当前图像所耗费的时间,继续等待指定的时间以使得下一张图像能够
            // 按照时间戳被送入到SLAM系统中进行跟踪
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            //std::cout << "T: " << T << std::endl;
            //std::cout << "ttrack: " << ttrack << std::endl;

            if(ttrack<T) {
                //std::cout << "usleep: " << (dT-ttrack) << std::endl;
                usleep((T-ttrack)*1e6); // 1e6
            }
        }
            // step 4.7 保存每一个序列的子图
        if(seq < num_seq - 1)
        {
            string kf_file_submap =  "./SubMaps/kf_SubMap_" + std::to_string(seq) + ".txt";
            string f_file_submap =  "./SubMaps/f_SubMap_" + std::to_string(seq) + ".txt";
            SLAM.SaveTrajectoryEuRoC(f_file_submap);
            SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file_submap);

            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }

    }
    // Stop all threads
    //* step 5 如果所有的图像都预测完了,那么终止当前的SLAM系统
    SLAM.Shutdown();

    //* step 6 保存相机轨迹
    // Save camera trajectory
    if (bFileName) // 如果给名字了就放到名字下的文件夹中
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else           //否则就按照默认名字
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }
    //* TODO：没有计算平均耗时，没有保存成TUM格式的选项

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t*1e-9);

        }
    }
}
