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
//* 双目融合IMU 跑EUROC数据集
#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>


#include<System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

using namespace std;
//加载图像、加载IMU数据声明
void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);


int main(int argc, char **argv)
{   // 同样支持输入多序列
    //* step 0, 检查输入量不小于5
    if(argc < 5)
    {
        cerr << endl << "Usage: ./stereo_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) " << endl;
        return 1;
    }
    //* step 1 判断输入序列的个数，并初始化存储各类信息的容器
    const int num_seq = (argc-3)/2;              // 输入序列的个数
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageLeft;       // 存储左目图像
    vector< vector<string> > vstrImageRight;      // 存储右目图像
    vector< vector<double> > vTimestampsCam;      // 存储时间戳
    vector< vector<cv::Point3f> > vAcc, vGyro;    // 速度、陀螺仪
    vector< vector<double> > vTimestampsImu;      // IMU时间戳
    vector<int> nImages;                          // 存取图像的容器
    vector<int> nImu;                             // 存储IMU的容器
    vector<int> first_imu(num_seq,0);             // 指定值初始化，first_imu被初始化为包含num_seq个值为0的int型的容器
    //利用输入序列的数量初始化所有存储容器
    vstrImageLeft.resize(num_seq);
    vstrImageRight.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);
    //* step 3 开始按照各个序列依次进行图像数据、IMU数据加载
    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2*seq) + 3]);
        string pathTimeStamps(argv[(2*seq) + 4]);
        // euroc数据结构的地址设置
        string pathCam0 = pathSeq + "/mav0/cam0/data";   
        string pathCam1 = pathSeq + "/mav0/cam1/data";
        string pathImu = pathSeq + "/mav0/imu0/data.csv";
        //加载左右目图像，左右目图像的名字，以及时间戳
        LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;
        // 加载IMU时间，三轴加速度，三轴角速度
        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]); 
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first

        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered
    }

    // Read rectification parameters
    // 读取配置文件，单目_euroc没有这步
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    // Vector for tracking time statistics
    // 记录每一帧图像处理时间的容器
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //* step 4 初始化SLAM系统，跟踪，局部建图、回环、可视化线程，设置为IMU_STEREO模式，设置可视化为false
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, false); // 利用类的构造函数初始化
    //* step 5 追踪每个序列中的每一帧图像
    cv::Mat imLeft, imRight;
    for (seq = 0; seq<num_seq; seq++) // 加入只有一个序列 seq = 0
    {
        // Seq loop
        vector<ORB_SLAM3::IMU::Point> vImuMeas; // 装IMU数据的容器
        double t_rect = 0.f;
        double t_resize = 0.f;
        double t_track = 0.f;
        int num_rect = 0;
        int proccIm = 0;
        // main loop
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)  // ni 是 seq序列中 图像的数量
        {
            // Read left and right images from file
            // step 5.1 读取左右目图像，并不对图像进行任何改变
            imLeft = cv::imread(vstrImageLeft[seq][ni],cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[seq][ni],cv::IMREAD_UNCHANGED);
            // step 5.2 检查图形合理性
            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageLeft[seq][ni]) << endl;
                return 1;
            }

            if(imRight.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageRight[seq][ni]) << endl;
                return 1;
            }

            double tframe = vTimestampsCam[seq][ni]; //当前帧图像的时间戳

            // Load imu measurements from previous frame 从前一帧加载IMU测量值，即加载一张图像，然后加载前一帧的IMU 保证之前的图像都有对应的IMU值
            // step 5.3 根据图像的时间戳，加载IMU数据
            vImuMeas.clear();  // 容器元素清零，只是size为0，但是申请的内存没被释放，capacity仍然保持原来的大小

            if(ni>0) // 如果ni>0， 即此次序列的第一张图像已经加载完
                // 如果当前的IMU时间戳小于当前帧图像的时间戳
                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni]) // while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
                {
                    // 加入一个Point类对象，包含三轴加速度三轴角速度以及时间戳
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++; //推入了一个IMU数据，当前序列+1,下次将读取下一个时间戳的数值。
                }
            // step 4.4 开始计时
    //#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //#else
            //std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    //#endif

            // Pass the images to the SLAM system
            // step 5.5 追踪当前帧图像，输入形参依次为：左目图像、右目图像，当前帧时间戳，IMU数据
            SLAM.TrackStereo(imLeft,imRight,tframe,vImuMeas);

    //#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    //#else
            //std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    //#endif

#ifdef REGISTER_TIMES
            t_track = t_rect + t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack; // 记录当前帧的追踪时间

            // step 5.6 Wait to load the next frame  根据图像时间戳中记录的两张图像之间的时间和现在追踪当前图像所耗费的时间,继续等待指定的时间以使得下一张图像能够
            // 按照时间戳被送入到SLAM系统中进行跟踪
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6
        }

        if(seq < num_seq - 1)  // 如果当前序列不是最后一个序列，在每一个序列完成后，就输出提醒，要变更数据集了
        //- 和单目_euroc相比，没有保存每个数据集子图的过程
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }


    }
    // Stop all threads
    //* step 6 如果所有的图像都预测完了,那么终止当前的SLAM系统
    SLAM.Shutdown();


    // Save camera trajectory
    //* step 7 保存相机轨迹
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }
    //TODO：虽然记录了时间但是没有对时间进行分析，需要后续补充对于时间的分析

    return 0;
}

/**
 * @description:  加载图像的函数
 * @param {string} &strPathLeft  左目图像的地址
 * @param {string} &strPathRight 右目图像的地址
 * @param {string} &strPathTimes 时间戳的地址
 * @param {vector<string>} &vstrImageLeft   存储左目图像名字的容器
 * @param {vector<string>} &vstrImageRight  存储右目图像名字的容器
 * @param {vector<double>} &vTimeStamps     存储时间戳的容器
 * @return {*}
 */
void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());  // 打开时间戳文件
    // 重置容器的空间大小
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    // 时间没结束之前
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png"); // 名字， 加入容器里
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);   // 时间戳 加入容器里

        }
    }
}
/**
 * @brief: 
 * @param [string] &strImuPath:
 * @param [vector<double>] &vTimeStamps:
 * @param [vector<cv::Point3f>] &vAcc:
 * @param [vector<cv::Point3f>] &vGyro:
 * @return [*]
 */
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}
