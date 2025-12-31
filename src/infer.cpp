/*
    包含读取雷达和摄像头数据，进行检测和跟踪.并对数据进行发布。
*/
#define CC_APP_VERSION_NAME  "INFER_NODE"
#define CC_APP_MAJOR_VERSION "0"
#define CC_APP_MINOR_VERSION "3"
#define CC_APP_PATCH_VERSION "2"
#include <infer.h>
#include <iostream>
#include<string.h>
#include "ff_decode.hpp"
#include "detector.hpp"
#include "resnet.hpp"
#include "device_params_config.h"
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>
// #include "// diagnostic.hpp"
#include "img_score.hpp"  // 图像质量检测
using namespace std;


class  InferDet{
  public:
    void  load_param(image_transport::Publisher pub_img, ros::Publisher  pub_tracker, ros::Publisher pub_fps, string camer_type , string camera_direction, int vechile_color_rate, int abandon_rate);
    void  load_model(std::vector<std::string> model_paths);
    void  setWriteParam(std::string byte_track_config_file, 
                       bool write_flag,
                       std::string write_path,
                       int min_points_len);

    int   processRadarCamera(int index);
    void  processHz();
    void  vehicleColor();
    void  abandonDetect();

  private:
    cv::Mat img_src ;
    ros::Publisher pub_fps;
    ros::Publisher pub_tracker;
    image_transport::Publisher pub_img;

    double img_time_sec = 0.0;
    double img_time_nsec = 0.0;
    double radar_time_sec = 0.0;
    double radar_time_nsec = 0.0;
    std::string camera_type;
    std::string camera_direction;
    int vehicle_color_rate = 10;           ///< Vehicle color processing rate 
    int abandon_rate = 5;                  ///< Abandoned object processing rate
    int min_points_len = 30;               ///< Min track points to save
    bool write_flag = false;               ///< Enable tracking data saving
    std::string write_path = "/tmp/";      ///< Path for saved trajectories
    std::string byte_track_config_file;    ///< ByteTrack configuration path
    bool draw_tracker_flag = true;         ///< Enable track visualization

    int publish_hz = 0 ;

    YoloV8_det detector ;
    YoloV8_det abandon_detector;
    RESNET vehicle_color_detector;

    // 每个进程都加载三个任务，1，主的检测跟踪任务，2，车型颜色任务，3，车牌识别任务。
    std::mutex mtx;
    SafeQueue<ModelInputData> vehicleColorQueue;  // 车颜色分类队列
    SafeQueue<ModelInputData> plateRecQueue;  // 车牌识别队列
    SafeQueue<AbandonInputData> abandonRecQueue;  // 抛洒物识别队列

    SafeQueue<VehicleColorResult> vehicleColorResultQueue;  // 车辆颜色识别队列
    SafeQueue<PlateRecResult> plateRecResultQueue;  // 车牌结果队列4
    SafeQueue<DetectorRetDatas> abandonResultQueue;  // 车牌结果队列4

    // Diagnostic diagnostic;   // 错误码上报

  };


  void  InferDet::load_param(image_transport::Publisher _pub_img, ros::Publisher  _pub_tracker, ros::Publisher _pub_fps , string _camera_type, string _camera_direction, int _vechile_color_rate, int _abandon_rate){

    // diagnostic.init("Infer Model", "Infer Camera", rvf::system::ModuleCode::kInfer);

    pub_img = _pub_img;
    pub_tracker = _pub_tracker;
    pub_fps = _pub_fps;
    camera_type = _camera_type;
    camera_direction = _camera_direction;
    vehicle_color_rate = _vechile_color_rate;
    abandon_rate = _abandon_rate;    
    ROS_INFO("load param success");
  }


  void  InferDet::load_model(std::vector<std::string> model_paths){


    if (access(model_paths[0].c_str(), F_OK) != 0 || access(model_paths[0].c_str(), F_OK) != 0 || access(model_paths[3].c_str(), F_OK) != 0 ) {
        // diagnostic.PubDiagnosticData(ERROR_CODE_LEVEL::ERROR, rvf::system::KeyCode::kModelLoadFailed, "DetectorModelFileNotExists"  );
        ROS_ERROR("Error: Detector model file %s not exists", model_paths[0].c_str());
    }
    // initialize net
    detector.Init(model_paths[0]);
    vehicle_color_detector.Init(model_paths[1]);
    ROS_INFO("Loading... Abandon Model %s", model_paths[3].c_str());
    abandon_detector.Init(model_paths[3]);
    ROS_INFO("Load... Abandon Model Done");
    ROS_INFO("init model success");
}

void InferDet::setWriteParam(std::string byte_track_config_file_, bool write_flag_, std::string write_path_, int min_points_len_){
    byte_track_config_file = byte_track_config_file_;
    write_flag = write_flag_;
    write_path = write_path_;
    if(min_points_len_ > 0 ) min_points_len = min_points_len_;
}

void  InferDet::processHz(){
    std_msgs::Float32  hz_data;
    ros::Rate rate(1 / 5.0);

    while (true) {
        try {
            rate.sleep();
            std::lock_guard<std::mutex> lock(mtx);
            if (publish_hz >= 0) {
                hz_data.data = publish_hz / 5.0;
                pub_fps.publish(hz_data);
                publish_hz = 0;

                if(camera_direction != "" && camera_type != "" && hz_data.data == 0){
                    // diagnostic.PubDiagnosticData(ERROR_CODE_LEVEL::WARN, rvf::system::KeyCode::kNoInferData, "NoInferData-" + camera_direction + "-" + camera_type);
                }
                ROS_INFO("%s %s publish_hz: %f", camera_direction.c_str(), camera_type.c_str(),  hz_data.data);
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Exception in process_hz: %s", e.what());
        }
    }
}


int  InferDet::processRadarCamera(int index){
 
    ROS_INFO("processRadarCamera thread runing...");

    cv::Mat mat_receive;
    sensor_msgs::ImageConstPtr msg_img;
    infer_nodelet::RadarTrackObjectProject::ConstPtr msg_track; 
    
    // 跟踪部分
    bytetrack_params params;
    bytetrack_yaml_parse(byte_track_config_file, params);
    BYTETracker bytetrack(params, write_flag, write_path, min_points_len,  camera_type, camera_direction);
    
    int rec_index = 1; // 其他目标无需要实时检测，计数器
    int detect_blackscreen_rate = 14 * 60;
    auto log_time = std::chrono::system_clock::now();  //    记录一次推理的开始时间
    std::vector<DetectorRetData> abandon_results;   // 主线程发的是这一段时间内检测的重复数据.
    bool print_diff_time = true;  // 是否打印时间差

    ROS_INFO("processRadarCamera enter loop...");
    int lost_camera_count = 0;
    int lost_radar_count = 0;

    while(true){
        auto m_ = imgQueue[index].Consume(msg_img);
        if(m_){
            img_time_sec = msg_img -> header.stamp.sec;
            img_time_nsec = msg_img -> header.stamp.nsec;
            lost_camera_count = 0;

        }
        else{
            usleep(2000);
            lost_camera_count ++;
            if(lost_camera_count == 600 && camera_direction != "" && camera_type != "" ){
                ROS_WARN("No Camera Data %s %s %d", camera_direction.c_str(), camera_type.c_str(), lost_camera_count);
                // diagnostic.PubDiagnosticData(ERROR_CODE_LEVEL::WARN, rvf::system::KeyCode::kNoCameraData, "NoCameraData-" + camera_direction + "-" + camera_type);
                lost_camera_count = 0;
            }
            continue;
        }
        auto once_start_time = std::chrono::system_clock::now();

        try {
            mat_receive =  cv_bridge::toCvShare(msg_img, "bgr8")->image ;
            }
        catch( cv_bridge::Exception& e )
            {
                // diagnostic.PubDiagnosticData(ERROR_CODE_LEVEL::ERROR, rvf::system::KeyCode::kImgCodeError, "ImageDecodeError-" + camera_direction + "-" + camera_type);
                ROS_ERROR( "Could not convert from '%s' to 'bgr8'.", msg_img->encoding.c_str() );
            }

        if (mat_receive.empty()){
            ROS_ERROR("Error: Could not parse image") ;
            // diagnostic.PubDiagnosticData(ERROR_CODE_LEVEL::ERROR, rvf::system::KeyCode::kImgCodeError, "ImageDecodeError-" + camera_direction + "-" + camera_type);

            continue;
        }
        else{
            mat_receive.copyTo(img_src);
            int batch_size = detector.batch_size;
            vector<bm_image> batch_imgs;
            vector<YoloV8BoxVec> boxes;
            bm_image bmimg;

            // 此处需要升级后才能保证正常 0.5.1
            try {
                assert(0 == bm_image_create(detector.handle, mat_receive.rows,mat_receive.cols, FORMAT_BGR_PACKED, DATA_TYPE_EXT_1N_BYTE, &bmimg));
                bm_image_alloc_dev_mem(bmimg, BMCV_IMAGE_FOR_IN);
                void *buffer[1]={static_cast<void*>(mat_receive.data)};
                assert( 0 == bm_image_copy_host_to_device(bmimg, buffer));
                batch_imgs.push_back(bmimg);
                CV_Assert(0 == detector.Detect(batch_imgs, boxes, index));
            }
            catch(...){
                ROS_ERROR("convert to bmimage error");
                continue;
            }

            std::vector<DetectorRetData>  res; // 检测结果
            for(int i = 0; i < batch_size; i++){
                for (int j=0; j<boxes[i].size(); j++) {
                    YoloV8Box box = boxes[i][j];
                    DetectorRetData single_det_result;
                    
                    // 封装
                    single_det_result.label = box.class_id ;
                    single_det_result.confidence = box.score;
                    single_det_result.xmin  = (int)box.x1;
                    single_det_result.ymin  = (int)box.y1;
                    single_det_result.xmax  = (int)(box.x2);
                    single_det_result.ymax  = (int)(box.y2);

                    if (single_det_result.xmin < 0 || single_det_result.ymin < 0 || single_det_result.xmax > img_src.cols || single_det_result.ymax > img_src.rows) continue;
                    res.push_back(single_det_result);
                }
            }
            CV_Assert(0 == bm_image_destroy(batch_imgs[0]));

            // 跳帧检测抛洒物 ，将检测结果和原图一并丢入检测线程，用于过滤误报的抛洒物
            rec_index ++;
            if(rec_index % abandon_rate == 0 ){
                AbandonInputData input_abandon_data;
                input_abandon_data.im = mat_receive.clone();
                input_abandon_data.data = res;
                abandonRecQueue.Produce(std::move(input_abandon_data));
            }

            // 图像相关检测
            if(1){
                // 黑屏检测
                if(rec_index % detect_blackscreen_rate == 0){
                    // cv::Mat black_im = cv::imread("/black.jpg");
                    double black_score = signalDetect(mat_receive.clone());
                    ROS_INFO("%s %s DETECT BLACK SCREEN %f" ,camera_direction.c_str(), camera_type.c_str(), black_score);
                    if (black_score > 0.8){
                        // diagnostic.PubDiagnosticData(ERROR_CODE_LEVEL::WARN, rvf::system::KeyCode::kBlackScreen, "BlackScreen");
                    }
                } 

                // 遮挡检测
                if(rec_index % detect_blackscreen_rate == 0){
                    double block_score = blockDetect(mat_receive.clone());
                    ROS_INFO("%s %s DETECT BLOCK SCREEN %f",camera_direction.c_str(), camera_type.c_str(),block_score);
                    if (block_score > 0.8){
                        // diagnostic.PubDiagnosticData(ERROR_CODE_LEVEL::WARN, rvf::system::KeyCode::kBlockScreen, "BlockScreen");
                    }
                }

                // 雪花检测
                if(rec_index % detect_blackscreen_rate == 0){
                    double snow_score = snowNoiseDetect(mat_receive.clone());
                    ROS_INFO("%s %s DETECT SNOW SCREEN %f",camera_direction.c_str(), camera_type.c_str(),snow_score);
                    if (snow_score > 0.8){
                        // diagnostic.PubDiagnosticData(ERROR_CODE_LEVEL::WARN, rvf::system::KeyCode::kSnowScreen, "SnowScreen");
                    }

                } 

                // 过暗检测
                if(rec_index % detect_blackscreen_rate == 0){
                    double bright_score = brightnessDetect(mat_receive.clone());
                    ROS_INFO("%s %s DETECT BRIGHT SCREEN %f",camera_direction.c_str(), camera_type.c_str(), bright_score);
                    if (bright_score > 0.8){
                        // diagnostic.PubDiagnosticData(ERROR_CODE_LEVEL::WARN, rvf::system::KeyCode::kBrightnessScreen, "BrightScreen");
                    }
                }

                // 模糊检测
                if(rec_index % detect_blackscreen_rate == 0){
                    double blur_score = sharpnessDetect(mat_receive.clone());    
                    ROS_INFO("%s %s DETECT BLUR SCREEN %f",camera_direction.c_str(), camera_type.c_str(), blur_score);
                    if (blur_score > 0.8){
                        // diagnostic.PubDiagnosticData(ERROR_CODE_LEVEL::WARN, rvf::system::KeyCode::kBlurScreen, "BlurScreen");
                    }
                }   

                // 日夜检测
                if(rec_index % detect_blackscreen_rate == 0){
                    bool day_or_night = DayOrNight(mat_receive.clone());
                    if (day_or_night){
                        ROS_INFO("%s %s DAY",camera_direction.c_str(), camera_type.c_str());
                    }
                    else{
                        ROS_INFO("%s %s NIGHT",camera_direction.c_str(), camera_type.c_str());
                    }
                }
            }


            // draw radar
            if(1){
                auto t_ = trackQueue[index].Consume(msg_track);
                if(t_ ) {
                    radar_time_sec = msg_track->header.stamp.sec;
                    radar_time_nsec = msg_track->header.stamp.nsec;

                    lost_radar_count = 0;

                    int time_diff = img_time_sec * 1000 + img_time_nsec / 1000000  - radar_time_sec * 1000 - radar_time_nsec/ 1000000;
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

                    //  每分钟的打印下时间差
                    if (print_diff_time){
                        if(time_diff < -400 || time_diff > 400) {
                            ROS_WARN("Diff Time: %s %s  %d img: %lf  radar: %lf ",camera_direction.c_str(), camera_type.c_str(),  time_diff, (img_time_sec * 1000 + img_time_nsec / 1000000 -ms ),(radar_time_sec * 1000 + radar_time_nsec/ 1000000 -ms ));
                            }
                        else{
                            ROS_INFO("Diff Time: %s %s  %d img: %lf  radar: %lf ",camera_direction.c_str(), camera_type.c_str(), time_diff, (img_time_sec * 1000 + img_time_nsec / 1000000 -ms ),(radar_time_sec * 1000 + radar_time_nsec/ 1000000 -ms ));
                        }
                        print_diff_time = false;
                    }

                    for (unsigned int i =0 ;i < msg_track->objects.size(); i ++){
                        if(camera_type == "long"){
                            auto x = msg_track->objects[i].x_pixel_norm_long * img_src.cols;
                            auto y = msg_track->objects[i].y_pixel_norm_long * img_src.rows;

                            if (x > 0 && y > 0){
                                cv::circle(img_src,cv::Point (x, y), 7, cv::Scalar(0, 0, 255), -1);
                            }
                        }
                        else{
                            auto x = msg_track->objects[i].x_pixel_norm_short * img_src.cols;
                            auto y = msg_track->objects[i].y_pixel_norm_short * img_src.rows;
                            if (x > 0 && y > 0){
                                cv::circle(img_src,cv::Point (x, y), 7, cv::Scalar(0, 0, 255), -1);
                            }
                        }
                    }
                }

                else{
                    lost_radar_count ++;
                    if(lost_radar_count == 1000 && camera_direction != "" && camera_type != "" ){
                        ROS_WARN("No Radar Data %s %s %d", camera_direction.c_str(), camera_type.c_str(), lost_radar_count);
                        // diagnostic.PubDiagnosticData(ERROR_CODE_LEVEL::WARN, rvf::system::KeyCode::kNoRadarData, "NoRadarData");
                        lost_radar_count = 0;
                    }
                }
            } // end draw
        
            std::vector<STrack> output_stracks = bytetrack.update(res); // 

            // 用 while的原因是，有多个颜色需要识别，一次性都拿出来.
            while(true){
                VehicleColorResult vehicle_color_result;
                auto _c = vehicleColorResultQueue.Consume(vehicle_color_result);
                if(!_c) break;
                bytetrack.updateVehicleColor(vehicle_color_result.tracker_id, vehicle_color_result.vehicle_color, vehicle_color_result.confidence);
            }

            // // test 调试检测框不准
            // for(int _i = 0; _i < res.size(); _i++){
            
            //     cv::Scalar _color = colors[3];
            //     int _x0 = max(0, (int)res[_i].xmin);
            //     int _y0 = max(0, (int)res[_i].ymin);
            //     int _w = max(0, (int)(res[_i].xmax - res[_i].xmin));
            //     int _h = max(0, (int)(res[_i].ymax - res[_i].ymin));
            //     cv::Rect _r= cv::Rect(_x0,_y0,_w,_h);
            //     cv::rectangle(img_src, _r, _color, 2);                             
            // }

            infer_nodelet::ImageDetectObject  tracker_msg;
            infer_nodelet::ImageDetectObjectSingle  single_msg;
            int objects_number = output_stracks.size();
            for (auto bbox : output_stracks) {
                int _id = bbox.track_id;

                int x0 = max(0, (int)bbox.det_box[0]);
                int y0 = max(0, (int)bbox.det_box[1]);
                int w = max(0, (int)bbox.det_box[2]);
                int h = max(0, (int)bbox.det_box[3]);
                cv::Rect r= cv::Rect(x0,y0,w,h);
                if (x0 <= 0 || y0 <= 0 || x0 > img_src.cols || x0 + w > img_src.cols || w <= 0 || h <= 0 || y0 > img_src.rows || y0 + h > img_src.rows){
                    // ROS_WARN("detector box : %d %d %d %d %d %d ", x0, y0, w, h, img_src.cols, img_src.rows );
                    continue;
                }

                // 画出每条轨迹
                if(draw_tracker_flag){
                    int track_x = 0;
                    int track_y = 0;
                    for(unsigned track_point_len = 0; track_point_len < bbox.track_points.size(); track_point_len++){                   
                        if(track_point_len == 0){
                            track_x = (int)(bbox.track_points[track_point_len][0]  + bbox.track_points[track_point_len][2] /2 );
                            track_y = (int)(bbox.track_points[track_point_len][1]  + bbox.track_points[track_point_len][3] /2 );
                        }
                        else{
                            auto _x = (int)(bbox.track_points[track_point_len][0]  + bbox.track_points[track_point_len][2] /2 );
                            auto _y = (int)(bbox.track_points[track_point_len][1]  + bbox.track_points[track_point_len][3] /2 );
                            cv::line(img_src, cv::Point(track_x, track_y), cv::Point(_x, _y),colors[bbox.class_id],2,8);
                            track_x = _x;
                            track_y = _y;
                        }
                    }
                }

                // 将颜色进行判断 取下半区域
                if(bbox.class_id > 2 &&  r.area() > 8000  && !bbox.color_lock ) {
                // if(bbox.class_id > 2 && !bbox.color_lock ) {
                    cv::Mat crop_vehicle_img = mat_receive(cv::Rect(r.x, r.y, r.width, r.height));
                    ModelInputData input_vehicle_data;
                    input_vehicle_data.im = crop_vehicle_img.clone();
                    input_vehicle_data.tracker_id = _id;
                    vehicleColorQueue.Produce(std::move(input_vehicle_data));
                }
                // 单轨迹数据处理完毕
                
                cv::Scalar color = colors[bbox.class_id];
                cv::rectangle(img_src, r, color, 2);    
                cv::putText(img_src, std::to_string(_id) + "-" + to_string(bbox.class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                
                // 封装发送数据
                single_msg.x_pixel_norm = std::clamp((float)r.x  / img_src.cols, float(0),float(1));
                single_msg.y_pixel_norm = std::clamp((float)r.y / img_src.rows,float(0),float(1));
                single_msg.w_pixel_norm = std::clamp((float)r.width  / img_src.cols, float(0),float(1));
                single_msg.h_pixel_norm = std::clamp((float)r.height / img_src.rows ,float(0),float(1));
                single_msg.target_type  = bbox.class_id;
                single_msg.id = _id;
                single_msg.color        = bbox.vehicle_color;
                single_msg.plate_number = bbox.plate_number;
                single_msg.plate_confid = bbox.plate_confid;
                single_msg.plate_color  = bbox.plate_color;
                tracker_msg.objects.push_back(single_msg);
            }
            
            // 更新抛洒物路障路牌， 这不是每帧检测，所以在间隔时间内发送一样的数据.
            if(1){
                DetectorRetDatas abandon_data;
                while(true){
                    auto _abandon = abandonResultQueue.Consume(abandon_data);
                    if(!_abandon) break;
                    else abandon_results = abandon_data.data;  // 只有数据发过来了才更新，否则沿用.
                }

                for(unsigned _abandon_index = 0; _abandon_index < abandon_results.size(); _abandon_index ++){
                    cv::Rect r= cv::Rect(abandon_results[_abandon_index].xmin, abandon_results[_abandon_index].ymin, \
                        abandon_results[_abandon_index].xmax - abandon_results[_abandon_index].xmin, \
                            abandon_results[_abandon_index].ymax - abandon_results[_abandon_index].ymin);

                    single_msg.x_pixel_norm = std::clamp((float)abandon_results[_abandon_index].xmin / img_src.cols, float(0),float(1));
                    single_msg.y_pixel_norm = std::clamp((float)abandon_results[_abandon_index].ymin / img_src.rows,float(0),float(1));
                    single_msg.w_pixel_norm = std::clamp((float)(abandon_results[_abandon_index].xmax - abandon_results[_abandon_index].xmin)  / img_src.cols, float(0),float(1));
                    single_msg.h_pixel_norm = std::clamp((float)(abandon_results[_abandon_index].ymax - abandon_results[_abandon_index].ymin) / img_src.rows ,float(0),float(1));

                    /*11：动物，21：箱子，  22：轮胎，23：树枝，  31：施工车辆 ，32：施工人员 ，33：锥桶，  34：柱桶 ，35：路障， 36：路牌 */
                    if(abandon_results[_abandon_index].label == 1){
                        single_msg.target_type  = 33; 
                        single_msg.id = 65536;
                    }
                    else if(abandon_results[_abandon_index].label == 2){
                        single_msg.target_type  = 35; 
                        single_msg.id = 65537;
                    }
                    else if(abandon_results[_abandon_index].label == 3){
                        single_msg.target_type  = 21; 
                        single_msg.id = 65538;
                    }

                    single_msg.color        =  0;
                    single_msg.plate_confid =  0;
                    single_msg.plate_color  = 0;  // 目前暂无此功能
                    tracker_msg.objects.push_back(single_msg);
                    cv::Scalar color = colors[0];
                    cv::rectangle(img_src, r, color, 2);
                    cv::putText(img_src, "-other-", cv::Point(r.x , r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 3);
                    objects_number += 1;
                }
            }

            // 封装信息
            tracker_msg.header.seq = 1;
            tracker_msg.header.stamp.sec = img_time_sec;
            tracker_msg.header.stamp.nsec = img_time_nsec;
            tracker_msg.header.frame_id = "image";  
            tracker_msg.frame_seq += 1;
            tracker_msg.objects_number = objects_number;
            cv_bridge::CvImage brigeImg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_src);
            pub_img.publish(brigeImg.toImageMsg());
            pub_tracker.publish(tracker_msg);

            auto once_end_time = std::chrono::system_clock::now();
            if (once_end_time - log_time > std::chrono::milliseconds(20000)) {
                ROS_INFO("all infer consume: %ld ms" , std::chrono::duration_cast<std::chrono::milliseconds>(once_end_time - once_start_time).count());
                log_time = once_end_time;
                print_diff_time = true;

            }
            publish_hz+=1;
        }
        
    }        
return 0;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg,int& index){
    sensor_msgs::ImageConstPtr _msg = msg;
    imgQueue[index].Produce(std::move(_msg));
}

void trackCallback(const infer_nodelet::RadarTrackObjectProject::ConstPtr msg, int& index){
    infer_nodelet::RadarTrackObjectProject::ConstPtr _msg = msg;
    trackQueue[index].Produce(std::move(_msg));
}

void InferDet::vehicleColor(){

    ModelInputData input_data;

    while(true){
        auto m_ = vehicleColorQueue.Consume(input_data);
        if(m_){
            // 进行车型颜色检测.
            VehicleColorResult vehicle_color_ret;
            vector<pair<int, float>> cls_results;
	    /*
            vector<bm_image> cls_batch_imgs;
            vector<string> cls_batch_names;
            vector<pair<int, float>> cls_results;
            bm_image cls_bmimg;

            assert(0 == bm_image_create(detector.handle, input_data.im.rows,input_data.im.cols, FORMAT_BGR_PACKED, DATA_TYPE_EXT_1N_BYTE, &cls_bmimg));
            bm_image_alloc_dev_mem(cls_bmimg, BMCV_IMAGE_FOR_IN);
            void *buffer[1]={static_cast<void*>(input_data.im.data)};
            assert( 0 == bm_image_copy_host_to_device(cls_bmimg, buffer));
        
            cls_batch_imgs.push_back(cls_bmimg);
            CV_Assert(0 == vehicle_color_detector.Classify(cls_batch_imgs, cls_results));
            bm_image_destroy(cls_batch_imgs[0]);
	    */
            vector<cv::Mat> batch_imgs;
            batch_imgs.push_back(input_data.im);
            CV_Assert(0 == vehicle_color_detector.Classify(batch_imgs, cls_results));

            vehicle_color_ret.confidence = cls_results[0].second ;
            vehicle_color_ret.vehicle_color = cls_results[0].first + 1;
            vehicle_color_ret.tracker_id =  input_data.tracker_id;

	    // start to write color ret
	    std::string color_img_name = to_string(input_data.tracker_id) + "_" + to_string(vehicle_color_ret.vehicle_color) + "_" + to_string(vehicle_color_ret.confidence);
	    // cv::imwrite("/home/files/tmp/color_imgs/" + color_img_name + ".jpg", input_data.im);

            // 将车型颜色检测结果放入队列中.
	    if(vehicle_color_ret.vehicle_color > 0.7)
              vehicleColorResultQueue.Produce(std::move(vehicle_color_ret));

        }
        else{
            usleep(2000);
        }
    }               
}


void InferDet::abandonDetect(){

    AbandonInputData input_data;

    while(true){
        auto m_ = abandonRecQueue.Consume(input_data);
        if(m_){
            
            std::vector<DetectorRetData>  res_abandon ;
            std::vector<DetectorRetData>  res_abandon_filter ;
            int batch_size = abandon_detector.batch_size;
            vector<bm_image> batch_imgs;
            vector<YoloV8BoxVec> boxes;
            bm_image bmimg;
            cv::Mat mat_receive = input_data.im.clone();
            try {
                assert(0 == bm_image_create(abandon_detector.handle, mat_receive.rows,mat_receive.cols, FORMAT_BGR_PACKED, DATA_TYPE_EXT_1N_BYTE, &bmimg));
                bm_image_alloc_dev_mem(bmimg, BMCV_IMAGE_FOR_IN);
                void *buffer[1]={static_cast<void*>(mat_receive.data)};
                assert( 0 == bm_image_copy_host_to_device(bmimg, buffer));
                batch_imgs.push_back(bmimg);
                CV_Assert(0 == abandon_detector.Detect(batch_imgs, boxes, 0));
            }
            catch(...){
                ROS_ERROR("convert to bmimage error");
                continue;
            }
            CV_Assert(0 == bm_image_destroy(batch_imgs[0]));

            // boxes 2 res_abandon format
            for(int i = 0; i < batch_size; i++){
                for (int j=0; j<boxes[i].size(); j++) {
                    YoloV8Box box = boxes[i][j];
                    DetectorRetData single_det_result;
                    
                    single_det_result.label = box.class_id;
                    single_det_result.confidence = box.score;
                    single_det_result.xmin  = (int)box.x1;
                    single_det_result.ymin  = (int)box.y1;
                    single_det_result.xmax  = (int)(box.x2);
                    single_det_result.ymax  = (int)(box.y2);

                    if (single_det_result.xmin <= 0 || single_det_result.ymin <= 0 || single_det_result.xmax > img_src.cols || single_det_result.ymax > img_src.rows) continue;
                    res_abandon.push_back(single_det_result);
                }
            }

            for (size_t j = 0; j < res_abandon.size(); j++) {

                if (res_abandon[j].xmin <= 0 || res_abandon[j].ymin <= 0 || res_abandon[j].xmax >= input_data.im.cols || res_abandon[j].ymax >= input_data.im.rows){
                    ROS_ERROR("RES:  xmin %d ymin  %d xmax %d ymax %d", res_abandon[j].xmin, res_abandon[j].ymin, res_abandon[j].xmax, res_abandon[j].ymax);
                    continue;
                }

                bool match_filter = false;
                // 所有的图和原图进行相比较
                for( unsigned int _i = 0 ; _i < input_data.data.size() ; _i ++){
                    float iom = CalculateOverlap(res_abandon[j].xmin, res_abandon[j].ymin, res_abandon[j].xmax, res_abandon[j].ymax , \
                        input_data.data[_i].xmin, input_data.data[_i].ymin, input_data.data[_i].xmax, input_data.data[_i].ymax);

                        if (iom >0.1){
                            match_filter = true;
                            break;
                        }
                    }
                // 如果没有合并的，添加进来。
                if (! match_filter){
                    res_abandon_filter.push_back(res_abandon[j]);
                }
            } //end compare

            {
                DetectorRetDatas abandon_data;
                abandon_data.data = res_abandon_filter;
                abandonResultQueue.Produce(std::move(abandon_data));
            }  

        }
        else{
            usleep(2000);
        }
    }               
}



namespace infer_ns {
    class DataNodelet : public nodelet::Nodelet {
    public:
        DataNodelet() = default;

        private:
            virtual void onInit() {
                NODELET_INFO("DataNodelet initialized!");
                ROS_INFO("version: %s %s %s", CC_APP_MAJOR_VERSION, CC_APP_MINOR_VERSION ,CC_APP_PATCH_VERSION);

                //封装参数
                std::vector<InferParam> infer_params ;
                ros::NodeHandle& private_nh = getMTPrivateNodeHandle();  // 内部参数
                ros::NodeHandle& nh = getMTNodeHandle();                 // 全局参数

                
                // 构造，自动解析
                DeviceParamsConfig cfg(nh);
                ROS_INFO_STREAM("Version: " << cfg.meta.version);

                auto & mec = cfg.mec_info;
                ROS_INFO("MEC: name=%s, index=%d, ip=%s, manufacturer=%s",
                    mec.mec_name.c_str(), mec.mec_index, mec.mec_ip.c_str(),
                    mec.mec_manufacturer.c_str());

                for (size_t i = 0; i < mec.poles.size(); ++i)
                {
                    const auto &pole = mec.poles[i];
                    ROS_INFO(" Pole #%zu: name=%s, index=%d, alt=%.1f",
                        i, pole.pole_name.c_str(), pole.pole_index, pole.altitude);

                    // camer数量
                    for (size_t c = 0; c < pole.cameras.size(); ++c)
                    {
                        const auto &cam = pole.cameras[c];
                        ROS_INFO("  Camera #%zu: idx=%d, dir=%s, focal=%s, ip=%s, port=%d, user=%s, pass=%s, enc=%s, manuf=%s",
                            c, cam.camera_index, cam.direction.c_str(), cam.focal_type.c_str(),
                            cam.ip.c_str(), cam.port, cam.username.c_str(), cam.password.c_str(),
                            cam.encryption_type.c_str(), cam.manufacturer.c_str());


                            InferParam infer_param;
                            infer_param.camera_type = cam.focal_type.c_str();
                            infer_param.camera_direction = cam.direction.c_str();

                            infer_param.receive_img_topic   = "/" + pole.pole_name + "/" + cam.direction.c_str() + "/" +  cam.focal_type.c_str() + "_camera/image_raw";
                            infer_param.publish_img_topic   = "/" + pole.pole_name + "/" + cam.direction.c_str() + "/" +  cam.focal_type.c_str() + "_camera/image_detect";
                            infer_param.publish_img_result  = "/" + pole.pole_name + "/" + cam.direction.c_str() + "/" +  cam.focal_type.c_str() + "_camera/image_detect_object";
                            infer_param.publish_fps         = "/" + pole.pole_name + "/" + cam.direction.c_str() + "/" +  cam.focal_type.c_str() + "_camera/image_detect_object/fps_hz";
                            infer_param.receive_radar_topic = "/" + pole.pole_name + "/" + cam.direction.c_str() + "/radar/track_object_project";

                            ROS_INFO( "camera_type : %s" , infer_param.camera_type.c_str());
                            ROS_INFO( "camera_direction : %s" , infer_param.camera_direction.c_str());
                            ROS_INFO( "receive_img_topic : %s" , infer_param.receive_img_topic.c_str());
                            ROS_INFO( "publish_img_topic : %s" , infer_param.publish_img_topic.c_str());
                            ROS_INFO( "publish_img_result : %s" ,infer_param.publish_img_result.c_str());
                            ROS_INFO( "receive_radar_topic : %s" ,infer_param.receive_radar_topic.c_str());
                            infer_params.push_back(infer_param);
                    }
                }
            
                /*
                    InferParam infer_param;
                    infer_param.camera_type = "short";
                    infer_param.receive_img_topic   = "/k695_218/hikcam1/hikcam1/";
                    infer_param.publish_img_topic   = "test";
                    infer_param.publish_img_result  = "/test_ret";
                    infer_param.publish_fps         = "/xxxfps";
                    infer_param.receive_radar_topic = "object_project";

                    ROS_INFO( "receive_img_topic : %s" , infer_param.receive_img_topic.c_str());
                    ROS_INFO( "publish_img_topic : %s" , infer_param.publish_img_topic.c_str());
                    ROS_INFO( "publish_img_result : %s" ,infer_param.publish_img_result.c_str());
                    ROS_INFO( "receive_radar_topic : %s" ,infer_param.receive_radar_topic.c_str());
                    infer_params.push_back(infer_param);
           
                */
		
                std::string models_names[] = {"det_engine_name","color_engine_name","freid_engine_name","abandon_engine_name"};
                std::vector<std::string> models_paths;

                for (size_t i = 0; i < sizeof(models_names) / sizeof(models_names[0]); i++) {

                    std::string model_path;
                    if (private_nh.getParam(models_names[i], model_path)) {
                        models_paths.push_back(model_path);
                    } else {
                        ROS_ERROR("Parameter %s not set, using default value.", models_names[i].c_str());
                        models_paths.push_back("");

                    }
                    ROS_INFO("Engine name: %s", model_path.c_str());
                }

                int abandon_rate = 5, vechile_color_rate = 10, min_points_len = 30;
                std::string write_path = "/home/files/nfsroot/", byte_track_config_file = "";
                bool write_flag = false;
                
                private_nh.param("abandon_rate", abandon_rate, abandon_rate);
                private_nh.param("vechile_color_rate", vechile_color_rate, vechile_color_rate);
                private_nh.param("min_points_len", min_points_len, min_points_len);
                private_nh.param("write_path", write_path, write_path);
                private_nh.param("byte_track_config_file", byte_track_config_file, byte_track_config_file);
                private_nh.param("write_flag", write_flag, write_flag);

                for (int infer_thread_index = 0;  infer_thread_index < (int)infer_params.size() ; infer_thread_index ++){
                // for (int infer_thread_index = 0;  infer_thread_index < 1 ; infer_thread_index ++){

                    std::shared_ptr<InferDet> infer_node = std::make_shared<InferDet>();

                    image_transport::ImageTransport imgTransport(nh);
                    image_transport::Publisher pub_img = imgTransport.advertise(infer_params[infer_thread_index].publish_img_topic , 1);
                    ros::Publisher pub_tracker = nh.advertise<infer_nodelet::ImageDetectObject>(infer_params[infer_thread_index].publish_img_result,1);
                    ros::Publisher pub_fps =  nh.advertise< std_msgs::Float32>(infer_params[infer_thread_index].publish_fps,1);
                    // ros::Publisher pub_img = nh.advertise<sensor_msgs::Image>(resultimg, 1, true);   // 采用原始的发布方式
                    std::string camera_type = infer_params[infer_thread_index].camera_type;
                    std::string camera_direction = infer_params[infer_thread_index].camera_direction;
                    ROS_INFO("Loading engine");
                    infer_node->load_param(pub_img, pub_tracker, pub_fps, camera_type, camera_direction , vechile_color_rate, abandon_rate);
                    infer_node->setWriteParam(byte_track_config_file, write_flag, write_path, min_points_len);
                    infer_node->load_model(models_paths);
                    auto spawn_thread = [](auto func, auto&& obj) {
                        return std::thread{std::forward<decltype(func)>(func), obj};
                    };
                    
                    ROS_INFO("Start threading");
                    std::thread infer_thread(&InferDet::processRadarCamera, infer_node, infer_thread_index);
                    std::thread fps_thread = spawn_thread(&InferDet::processHz, infer_node);               // fps统计
                    std::thread vehicle_color_thread = spawn_thread(&InferDet::vehicleColor, infer_node);  //车型颜色
                    // std::thread plate_rec_thread = spawn_thread(&InferDet::plateRec, infer_node);       //车牌识别
                    std::thread abandon_rec_thread = spawn_thread(&InferDet::abandonDetect, infer_node);     //抛洒物识别


                    infer_thread.detach();
                    fps_thread.detach();
                    vehicle_color_thread.detach();
                    abandon_rec_thread.detach();
                    // plate_rec_thread.detach();

                    std::cout<< "------------------call topic: " << infer_params[infer_thread_index].receive_img_topic << std::endl;
                    sub_radars[infer_thread_index] = nh.subscribe<infer_nodelet::RadarTrackObjectProject>(infer_params[infer_thread_index].receive_radar_topic,1, boost::bind(&trackCallback, _1, infer_thread_index));
                    sub_imgs[infer_thread_index] = nh.subscribe<sensor_msgs::Image>(infer_params[infer_thread_index].receive_img_topic , 1, boost::bind(&imgCallback, _1, infer_thread_index));
                     }
            
            // spinner.spin();
        }
        ros::Subscriber sub_imgs[6];
        ros::Subscriber sub_radars[6];
    };
}


PLUGINLIB_EXPORT_CLASS(infer_ns::DataNodelet, nodelet::Nodelet)

