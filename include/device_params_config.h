#ifndef DEVICE_PARAMS_CONFIG_H_
#define DEVICE_PARAMS_CONFIG_H_

#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <string>
#include <vector>
#include <type_traits>  // for std::is_same_v in C++17

template<typename T>
T get_xml_value(const XmlRpc::XmlRpcValue &parent,
                const std::string &key,
                const T &default_val)
{
    if (!parent.hasMember(key))
    {
        ROS_WARN("Missing field '%s', use default.", key.c_str());
        return default_val;
    }
    const auto &child = parent[key];

    if constexpr (std::is_same_v<T, int>)
    {
        if (child.getType() == XmlRpc::XmlRpcValue::TypeInt)
            return static_cast<int>(child);
        ROS_WARN("Field '%s' not TypeInt, use default.", key.c_str());
        return default_val;
    }
    else if constexpr (std::is_same_v<T, double>)
    {
        if (child.getType() == XmlRpc::XmlRpcValue::TypeInt)
            return static_cast<double>(static_cast<int>(child));
        else if (child.getType() == XmlRpc::XmlRpcValue::TypeDouble)
            return static_cast<double>(child);
        ROS_WARN("Field '%s' not numeric, use default.", key.c_str());
        return default_val;
    }
    else if constexpr (std::is_same_v<T, std::string>)
    {
        if (child.getType() == XmlRpc::XmlRpcValue::TypeString)
            return static_cast<std::string>(child);
        ROS_WARN("Field '%s' not TypeString, use default.", key.c_str());
        return default_val;
    }
    else
    {
        ROS_WARN("Unsupported type for field '%s', use default.", key.c_str());
        return default_val;
    }
}


struct RadarInfo
{
    int radar_index = 0;
    std::string direction;    // "upstream"/"downstream"/...
    std::string ip;
    int port = 0;
    std::string protocol;     // e.g. "ext"
    double longitude = 0.0;
    double latitude = 0.0;
    double heading = 0.0;     // 单位：度
};

struct CameraInfo
{
    std::string manufacturer; // "Hikvision"/"Dahua"/...
    int camera_index = 0;
    std::string direction;    //  upstream / downstream /
    std::string focal_type;   // "long"/"short"
    std::string ip;
    int port = 0;
    std::string username;
    std::string password;
    std::string encryption_type;  
};

struct PoleInfo
{
    std::string pole_name;
    int pole_index = 0;
    double altitude = 0.0;
    std::vector<RadarInfo> radars;
    std::vector<CameraInfo> cameras;
};

struct MecInfo
{
    std::string mec_name;
    int mec_index = 0;
    std::string mec_ip;
    std::string mec_manufacturer;
    std::vector<PoleInfo> poles;
};

struct DeviceParamsMeta
{
    std::string version;
};


class DeviceParamsConfig
{
public:
    DeviceParamsMeta meta;
    MecInfo mec_info; 

    explicit DeviceParamsConfig(const ros::NodeHandle &nh)
    {
        loadFromParamServer(nh);
    }

private:

    void loadFromParamServer(const ros::NodeHandle &nh)
    {

        meta.version = nh.param<std::string>("device_params_meta/version", "unknown");


        XmlRpc::XmlRpcValue dev_params_val;
        if (!nh.getParam("device_params", dev_params_val))
        {
            ROS_ERROR("Failed to get 'device_params' from param server. Check YAML!");
            return; 
        }
        if (dev_params_val.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_ERROR("'device_params' is not a struct!");
            return;
        }

        parseMecInfo(dev_params_val, mec_info);
    }

    void parseMecInfo(XmlRpc::XmlRpcValue &val, MecInfo &mec)
    {
        mec.mec_name         = get_xml_value<std::string>(val, "mec_name", "unknown");
        mec.mec_index        = get_xml_value<int>(val, "mec_index", 9999);
        mec.mec_ip           = get_xml_value<std::string>(val, "mec_ip", "unknown");
        mec.mec_manufacturer = get_xml_value<std::string>(val, "mec_manufacturer", "unknown");

        // poles
        if (val.hasMember("poles"))
        {
            XmlRpc::XmlRpcValue &poles_val = val["poles"];
            if (poles_val.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                parsePoleArray(poles_val, mec.poles);
            }
        }
    }

    void parsePoleArray(XmlRpc::XmlRpcValue &pole_arr, std::vector<PoleInfo> &poles)
    {
        for (int i = 0; i < pole_arr.size(); ++i)
        {
            if (pole_arr[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_WARN("poles[%d] not a struct, skip.", i);
                continue;
            }
            XmlRpc::XmlRpcValue &pval = pole_arr[i];

            PoleInfo pole;
            pole.pole_name = get_xml_value<std::string>(pval, "pole_name", "unknown");
            pole.pole_index = get_xml_value<int>(pval, "pole_index", 0);
            pole.altitude = get_xml_value<double>(pval, "altitude", -1);

            // radars
            if (pval.hasMember("radars") && pval["radars"].getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                parseRadarArray(pval["radars"], pole.radars);
            }

            // cameras
            if (pval.hasMember("cameras") && pval["cameras"].getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                parseCameraArray(pval["cameras"], pole.cameras);
            }

            poles.push_back(pole);
        }
    }

    void parseRadarArray(XmlRpc::XmlRpcValue &rarr, std::vector<RadarInfo> &radars)
    {
        for (int i = 0; i < rarr.size(); ++i)
        {
            if (rarr[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_WARN("radars[%d] not a struct, skip.", i);
                continue;
            }
            XmlRpc::XmlRpcValue &rv = rarr[i];

            RadarInfo rd;
            rd.radar_index = get_xml_value<int>(rv, "radar_index", 0);
            rd.direction   = get_xml_value<std::string>(rv, "direction", "unknown");
            rd.ip          = get_xml_value<std::string>(rv, "ip", "unknown");
            rd.port        = get_xml_value<int>(rv, "port", 8089);
            rd.protocol    = get_xml_value<std::string>(rv, "protocol", "unknown");
            rd.longitude   = get_xml_value<double>(rv, "longitude", -1);
            rd.latitude    = get_xml_value<double>(rv, "latitude", -1);
            rd.heading     = get_xml_value<double>(rv, "heading", -1);

            radars.push_back(rd);
        }
    }

    void parseCameraArray(XmlRpc::XmlRpcValue &carr, std::vector<CameraInfo> &cams)
    {
        for (int i = 0; i < carr.size(); ++i)
        {
            if (carr[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_WARN("cameras[%d] not a struct, skip.", i);
                continue;
            }
            XmlRpc::XmlRpcValue &cv = carr[i];

            CameraInfo cam;
            cam.manufacturer     = get_xml_value<std::string>(cv, "manufacturer", "unknown");
            cam.camera_index     = get_xml_value<int>(cv, "camera_index", 0);
            cam.direction        = get_xml_value<std::string>(cv, "direction", "unknown");
            cam.focal_type       = get_xml_value<std::string>(cv, "focal_type", "unknown");
            cam.ip               = get_xml_value<std::string>(cv, "ip", "unknown");
            cam.port             = get_xml_value<int>(cv, "port", 554);
            cam.username         = get_xml_value<std::string>(cv, "username", "unknown");
            cam.password         = get_xml_value<std::string>(cv, "password", "unknown");
            cam.encryption_type  = get_xml_value<std::string>(cv, "encryption_type", "unknown");

            cams.push_back(cam);
        }
    }
};

#endif  // DEVICE_PARAMS_CONFIG_H_
