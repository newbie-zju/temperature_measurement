#include <stdio.h>
#include <iostream>
#include "get_temperature.h"
#include <vector>
#include <time.h>

using namespace std;

string getTime()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep) );
    return tmp;
}

class TemperatureInfo
{
public:
    int num;
    int id;
    float temperature;
    int key;

    TemperatureInfo(){}

    TemperatureInfo(int wPresetNo, int byRuleID, float fCurrTemperature)
    {
        num = wPresetNo;
        id = byRuleID;
        temperature = fCurrTemperature;
        key = num + 100*id;
    }
};

class TemperatureWriter
{
public:
    vector<TemperatureInfo> ti_vector;

    TemperatureWriter(){}

    void write(TemperatureInfo ti)
    {
        for (int i = 0; i < ti_vector.size() ; i++)
            if (ti_vector[i].key == ti.key)
                return;
        cout << "时间:" << getTime() << ", " <<"预置点:" << ti.num << ", " << "ID:"<< ti.id << ", " << "温度:"<< ti.temperature << endl;
        ti_vector.push_back(ti);
    }
    void clean()
    {
        ti_vector.clear();
    }
} tw;

BOOL CALLBACK MessageCallback(LONG lCommand, NET_DVR_ALARMER *pAlarmer, char *pAlarmInfo, DWORD dwBufLen, void* pUser)
{
    switch(lCommand)
    {
        case COMM_THERMOMETRY_ALARM:
        {
            NET_DVR_THERMOMETRY_ALARM struThermometryAlarm = {0};
            memcpy(&struThermometryAlarm, pAlarmInfo, sizeof(NET_DVR_THERMOMETRY_ALARM));
            if (0 == struThermometryAlarm.byRuleCalibType)
            {
                TemperatureInfo ti_tmp(struThermometryAlarm.wPresetNo, struThermometryAlarm.byRuleID, struThermometryAlarm.fCurrTemperature);
                tw.write(ti_tmp);
//            cout << "预置点:" << int(struThermometryAlarm.wPresetNo) << ", "\
//            << "ID:"<< int(struThermometryAlarm.byRuleID) << ", "\
//            << "温度:"<< float(struThermometryAlarm.fCurrTemperature) << endl;
            }
        }
            break;
        default:
            cout<<"error"<<endl;
            break;
    }

    return TRUE;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "get_temperature_node");
    ros::NodeHandle nh;
    string ip;
    if(!nh.getParam("ip", ip)) ip = "192.168.0.200";
    string username;
    if(!nh.getParam("username", username)) username = "admin";
    string passwd;
    if(!nh.getParam("passwd", passwd)) passwd = "zjucsc301";

    //---------------------------------------
    NET_DVR_Init();
    NET_DVR_SetConnectTime(2000, 1);
    NET_DVR_SetReconnect(10000, true);

    //---------------------------------------
    LONG lUserID;

    NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
    struLoginInfo.bUseAsynLogin = 0;

    strcpy((char *)struLoginInfo.sDeviceAddress,  (char *)ip.data());
    strcpy((char *)struLoginInfo.sUserName, (char *) username.data());
    strcpy((char *)struLoginInfo.sPassword,  (char *)passwd.data());
    struLoginInfo.wPort = 8000;

    NET_DVR_DEVICEINFO_V40 struDeviceInfoV40 = {0};

    lUserID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);
    if (lUserID < 0)
    {
        printf("Login failed, error code: %d\n", NET_DVR_GetLastError());
        NET_DVR_Cleanup();
        return 0;
    }

    //---------------------------------------
    NET_DVR_SetDVRMessageCallBack_V31(MessageCallback, NULL);

    //---------------------------------------
    LONG lHandle;
    NET_DVR_SETUPALARM_PARAM  struAlarmParam={0};
    struAlarmParam.dwSize=sizeof(struAlarmParam);

    lHandle = NET_DVR_SetupAlarmChan_V41(lUserID, & struAlarmParam);
    if (lHandle < 0)
    {
        printf("NET_DVR_SetupAlarmChan_V41 error, %d\n", NET_DVR_GetLastError());
        NET_DVR_Logout(lUserID);
        NET_DVR_Cleanup();
        return 0;
    }

    //---------------------------------------
    // wait
    ros::spin();

    //---------------------------------------
    if (!NET_DVR_CloseAlarmChan_V30(lHandle))
    {
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