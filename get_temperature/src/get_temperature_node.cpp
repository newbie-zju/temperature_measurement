#include <stdio.h>
#include <iostream>
#include "get_temperature.h"
#include <vector>
#include <time.h>
#include <fstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

string output_file;

Mat src;
Mat src_org;

void imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_ptr->image.copyTo(src);
}

string getTime() {
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&timep));
    return tmp;
}

class TemperatureInfo {
public:
    int num;
    int id;
    float temperature;
    int key;
    int level;

    TemperatureInfo() {}

    TemperatureInfo(int wPresetNo, int byRuleID, float fCurrTemperature, int byAlarmLevel) {
        num = wPresetNo;
        id = byRuleID;
        temperature = fCurrTemperature;
        key = num + 100 * id;
        level = byAlarmLevel;
    }
};

class TemperatureWriter {
public:
    vector<TemperatureInfo> ti_vector;
    string output_file;

    TemperatureWriter() {}

    TemperatureWriter(string output_file0) {
        output_file = output_file0;
    }

    void write(TemperatureInfo ti) {
        for (int i = 0; i < ti_vector.size(); i++)
            if (ti_vector[i].key == ti.key)
                return;

        ifstream ifile;
        ifile.open(output_file);
        if (!ifile) {
            cout << output_file << "不存在" << endl;
            ofstream ofile;
            ofile.open(output_file);
            ofile << endl;
            ofile.close();
            ifile.open(output_file);
            if (!ifile) {
                cout << "创建失败" << endl;
                return;
            } else {
                cout << "创建成功" << endl;
                ifile.close();
            }
        }

        ofstream ofile;
        ofile.open(output_file, ios::app);
        ofile << "时间:" << getTime() << ", " << "预置点:" << ti.num << ", " << "ID:" << ti.id << ", " << "温度:"
              << ti.temperature << "等级:" << ti.level << endl;
        cout << "时间:" << getTime() << ", " << "预置点:" << ti.num << ", " << "ID:" << ti.id << ", " << "温度:"
             << ti.temperature << "等级:" << ti.level << endl;
//        cout<<ti.level<<endl;
        if (ti.level == 1) {
            if (src.type() != 0) {
                string image_output_file = output_file;
                image_output_file = image_output_file.replace(image_output_file.find(".txt"), 4, + " time:"+
                                                              getTime() + " num:" + to_string(ti.num) + " id:" +
                                                              to_string(ti.id) + " t:" + to_string(ti.temperature) +
                                                              " l:" + to_string(ti.level) + ".jpg");
                cout << image_output_file << endl;
                imwrite(image_output_file, src);
                src = src_org;//clean
            } else {
                cout << "未收到红外线图像" << endl;
            }

        }

        ti_vector.push_back(ti);
    }

    void clean() {
        ti_vector.clear();
    }
} tw;

BOOL CALLBACK MessageCallback(LONG lCommand, NET_DVR_ALARMER *pAlarmer, char *pAlarmInfo, DWORD dwBufLen, void *pUser) {
    switch (lCommand) {
        case COMM_THERMOMETRY_ALARM: {
            NET_DVR_THERMOMETRY_ALARM struThermometryAlarm = {0};
            memcpy(&struThermometryAlarm, pAlarmInfo, sizeof(NET_DVR_THERMOMETRY_ALARM));
            if (0 == struThermometryAlarm.byRuleCalibType) {
                TemperatureInfo ti_tmp(struThermometryAlarm.wPresetNo, struThermometryAlarm.byRuleID,
                                       struThermometryAlarm.fCurrTemperature, struThermometryAlarm.byAlarmLevel);
                tw.write(ti_tmp);
//            cout << "预置点:" << int(struThermometryAlarm.wPresetNo) << ", "\
//            << "ID:"<< int(struThermometryAlarm.byRuleID) << ", "\
//            << "温度:"<< float(struThermometryAlarm.fCurrTemperature) << endl;
            }
        }
            break;
        default:
            cout << "error" << endl;
            break;
    }

    return TRUE;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "get_temperature_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it_(nh);
    image_transport::Subscriber image_sub_;
    string subscribed_topic;
    if (!nh.getParam("subscribed_topic", subscribed_topic)) subscribed_topic = "/hk_temperature_video";
    image_sub_ = it_.subscribe(subscribed_topic, 1, imageCb);
//    image_sub_ = it_.subscribe(subscribed_topic, 1, &PedstrianDetectionOpencv::imageCb, this);
    string ip;
    if (!nh.getParam("ip", ip)) ip = "192.168.0.200";
    string username;
    if (!nh.getParam("username", username)) username = "admin";
    string passwd;
    if (!nh.getParam("passwd", passwd)) passwd = "zjucsc301";
    if (!nh.getParam("output_file", output_file)) output_file = "~/Desktop/wl/result.txt";
    tw = TemperatureWriter(output_file);

    //---------------------------------------
    NET_DVR_Init();
    NET_DVR_SetConnectTime(2000, 1);
    NET_DVR_SetReconnect(10000, true);

    //---------------------------------------
    LONG lUserID;

    NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
    struLoginInfo.bUseAsynLogin = 0;

    strcpy((char *) struLoginInfo.sDeviceAddress, (char *) ip.data());
    strcpy((char *) struLoginInfo.sUserName, (char *) username.data());
    strcpy((char *) struLoginInfo.sPassword, (char *) passwd.data());
    struLoginInfo.wPort = 8000;

    NET_DVR_DEVICEINFO_V40 struDeviceInfoV40 = {0};

    lUserID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);
    if (lUserID < 0) {
        printf("Login failed, error code: %d\n", NET_DVR_GetLastError());
        NET_DVR_Cleanup();
        return 0;
    }

    //---------------------------------------
    NET_DVR_SetDVRMessageCallBack_V31(MessageCallback, NULL);

    //---------------------------------------
    LONG lHandle;
    NET_DVR_SETUPALARM_PARAM struAlarmParam = {0};
    struAlarmParam.dwSize = sizeof(struAlarmParam);

    lHandle = NET_DVR_SetupAlarmChan_V41(lUserID, &struAlarmParam);
    if (lHandle < 0) {
        printf("NET_DVR_SetupAlarmChan_V41 error, %d\n", NET_DVR_GetLastError());
        NET_DVR_Logout(lUserID);
        NET_DVR_Cleanup();
        return 0;
    }

    //---------------------------------------
    // wait
    ros::spin();

    //---------------------------------------
    if (!NET_DVR_CloseAlarmChan_V30(lHandle)) {
        printf("NET_DVR_CloseAlarmChan_V30 error, %d\n", NET_DVR_GetLastError());
        NET_DVR_Logout(lUserID);
        NET_DVR_Cleanup();
        return 0;
    }

    //---------------------------------------
    NET_DVR_Logout(lUserID);

    //---------------------------------------
    NET_DVR_Cleanup();
    return 0;
}