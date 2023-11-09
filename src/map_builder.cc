#include "map_builder.h"

#include <assert.h>
#include <iostream> 
#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>

#include "super_point.h"
#include "super_glue.h"
#include "read_configs.h"
#include "dataset.h"
#include "camera.h"
#include "frame.h"
#include "point_matching.h"
#include "map.h"
#include "g2o_optimization/g2o_optimization.h"
#include "timer.h"
#include "debug.h"

/*
 * MapBuilder类的构造函数。
 * 参数：
 *   configs - 配置参数对象
 * 成员变量：
 *   _shutdown - 是否关闭标志
 *   _init - 是否初始化标志
 *   _track_id - 跟踪ID
 *   _line_track_id - 线段跟踪ID
 *   _to_update_local_map - 是否更新局部地图标志
 *   _configs - 配置参数对象
 *   _camera - 相机对象
 *   _superpoint - SuperPoint对象
 *   _point_matching - 点匹配对象
 *   _line_detector - 线段检测对象
 *   _ros_publisher - ROS发布器对象
 *   _map - 地图对象
 *   _feature_thread - 特征提取线程
 *   _tracking_thread - 跟踪线程
 */

MapBuilder::MapBuilder(Configs& configs): _shutdown(false), _init(false), _track_id(0), _line_track_id(0), 
    _to_update_local_map(false), _configs(configs){
  _camera = std::shared_ptr<Camera>(new Camera(configs.camera_config_path));
  _superpoint = std::shared_ptr<SuperPoint>(new SuperPoint(configs.superpoint_config));
  if (!_superpoint->build()){
    std::cout << "Error in SuperPoint building" << std::endl;
    exit(0);
  }
  _point_matching = std::shared_ptr<PointMatching>(new PointMatching(configs.superglue_config));
  _line_detector = std::shared_ptr<LineDetector>(new LineDetector(configs.line_detector_config));
  _ros_publisher = std::shared_ptr<RosPublisher>(new RosPublisher(configs.ros_publisher_config));
  _map = std::shared_ptr<Map>(new Map(_configs.backend_optimization_config, _camera, _ros_publisher));

  _feature_thread = std::thread(boost::bind(&MapBuilder::ExtractFeatureThread, this));
  _tracking_thread = std::thread(boost::bind(&MapBuilder::TrackingThread, this));
}

/*
这段代码的逻辑如下：

1. 首先，调用相机类（_camera）的UndistortImage方法，对传入的左右图像进行去畸变处理，并将处理后的图像分别存储在image_left_rect和image_right_rect中。

2. 接下来，将image_left_rect和image_right_rect重新赋值给输入数据（data）的image_left和image_right，覆盖原始图像。

3. 然后，进入一个while循环，循环条件是_data_buffer中的数据个数大于等于3且_shutdown为false。在循环中，通过usleep函数暂停2毫秒。

4. 退出循环后，获取数据缓冲区（_data_buffer）的锁，将输入数据（data）压入数据缓冲区。

5. 最后，释放数据缓冲区的锁。

总体来说，该代码的逻辑是对输入数据进行去畸变处理，并将处理后的数据压入数据缓冲区。同时，使用一个while循环来控制数据缓冲区的大小，以避免数据堆积。*/

/**
 * 方法名: MapBuilder*****Input
 * 参数: data - 输入数据指针
 * 逻辑: 对输入的图像进行去畸变处理，将处理后的图像保存到data中
 *       然后将data放入数据缓冲区中，直到数据缓冲区大小达到3或程序被关闭
 *       最后释放互斥锁
 */

void MapBuilder::AddInput(InputDataPtr data){
  cv::Mat image_left_rect, image_right_rect;
  _camera->UndistortImage(data->image_left, data->image_right, image_left_rect, image_right_rect);
  data->image_left = image_left_rect;
  data->image_right = image_right_rect;

  while(_data_buffer.size() >= 3 && !_shutdown){
    usleep(2000);
  }

  _buffer_mutex.lock();
  _data_buffer.push(data);
  _buffer_mutex.unlock();
}


/**
 * @brief 提取特征的线程函数
 * @details
 * 这段代码是一个线程函数，主要用于从数据缓冲区中提取数据，并进行特征提取和追踪操作。
  代码首先进入一个无限循环，直到_shutdown标志为true才退出循环。
  循环的第一部分是检查数据缓冲区是否为空，如果为空，线程会休眠2000微秒（2毫秒），然后继续下一次循环。
  如果数据缓冲区不为空，线程会获取缓冲区中的第一个数据，并从缓冲区中移除。然后获取该数据的帧ID、时间戳、左右摄像机矫正后的图像。
  接下来，代码会构造一个帧对象，并根据是否是初始帧进行初始化操作。如果是初始帧，则调用Init函数进行初始化，并将该帧设置为上一帧。如果初始化成功，则发布该帧，并继续下一次循环。
  如果不是初始帧，则获取上一关键帧的特征，并提取当前帧的特征，并与上一关键帧进行匹配。然后将提取到的特征和匹配结果添加到当前帧对象中。
  接下来，创建一个TrackingData对象，用于保存追踪相关的数据。将当前帧、上一关键帧、匹配结果和输入数据赋值给TrackingData对象。
  然后，检查_tracking_data_buffer的大小，如果大于等于2，则线程会休眠2000微秒，直到其大小小于2为止。
  最后，线程会获取_tracking_mutex的锁，将TrackingData对象放入_tracking_data_buffer中，并释放锁。
  个循环不断从数据缓冲区中提取数据，进行特征提取和追踪操作，并将追踪结果保存到_tracking_data_buffer中。
 */

void MapBuilder::ExtractFeatureThread(){
  while(!_shutdown){
    if(_data_buffer.empty()){
      usleep(2000);
      continue;
    }
    InputDataPtr input_data;
    _buffer_mutex.lock();
    input_data = _data_buffer.front();
    _data_buffer.pop();
    _buffer_mutex.unlock();

    int frame_id = input_data->index;
    double timestamp = input_data->time;
    cv::Mat image_left_rect = input_data->image_left.clone();
    cv::Mat image_right_rect = input_data->image_right.clone();

    // 构建帧
    FramePtr frame = std::shared_ptr<Frame>(new Frame(frame_id, false, _camera, timestamp));

    // 初始化
    if(!_init){
      _init = Init(frame, image_left_rect, image_right_rect);
      _last_frame_track_well = _init;

      if(_init){
        _last_frame = frame;
        _last_image = image_left_rect;
        _last_right_image = image_right_rect;
        _last_keyimage = image_left_rect;
      }
      PublishFrame(frame, image_left_rect);
      continue;;
    }

    // 提取特征并跟踪最新关键帧
    FramePtr last_keyframe = _last_keyframe;
    const Eigen::Matrix<double, 259, Eigen::Dynamic> features_last_keyframe = last_keyframe->GetAllFeatures();

    std::vector<cv::DMatch> matches;
    Eigen::Matrix<double, 259, Eigen::Dynamic> features_left;
    std::vector<Eigen::Vector4d> lines_left;
    ExtractFeatureAndMatch(image_left_rect, features_last_keyframe, features_left, lines_left, matches);
    frame->AddLeftFeatures(features_left, lines_left);

    TrackingDataPtr tracking_data = std::shared_ptr<TrackingData>(new TrackingData());
    tracking_data->frame = frame;
    tracking_data->ref_keyframe = last_keyframe;
    tracking_data->matches = matches;
    tracking_data->input_data = input_data;
    
    while(_tracking_data_buffer.size() >= 2){
      usleep(2000);
    }

    _tracking_mutex.lock();
    _tracking_data_buffer.push(tracking_data);
    _tracking_mutex.unlock();
  }  
}

/**
 * 函数名称：MapBuilder::TrackingThread()
 * 函数功能：地图构建器的跟踪线程
 * @details
 * 这段代码是一个线程函数，用于执行地图构建的过程。代码中的主要逻辑如下：
  1. 进入一个无限循环，直到收到关闭信号为止。
  2. 检查_tracking_data_buffer 是否为空，如果为空则等待一段时间后继续循环。
  3. 锁定_tracking_mutex，从_tracking_data_buffer 中取出一个 TrackingDataPtr 对象，并将其从_buffer中移除。
  4. 从 tracking_data 中获取 frame、ref_keyframe、input_data 和 matches 等数据。
  5. 获取 input_data 中的时间戳和左右两侧的图像副本。
  6. 设置当前帧的姿态为上一帧的姿态。
  7. 定义一个函数 track_last_frame，用于跟踪上一帧，并在适当的条件下插入关键帧。
  8. 根据匹配点数目判断是否需要跟踪上一帧，若匹配点数目不足，则调用 track_last_frame 函数。
  9. 若匹配点数目足够，则调用 TrackFrame 函数进行帧间跟踪。
  10. 发布当前帧以及左侧图像。
  11. 判断帧间跟踪是否成功，并根据结果更新_last_frame_track_well 变量。
  12. 若帧间跟踪失败，则继续下一次循环。
  13. 设置当前帧的前一帧为参考关键帧，并更新_last_frame_track_well 变量。
  14. 检查当前帧是否应该成为关键帧，如果是则插入关键帧，并更新_last_keyimage 变量。
  15. 更新_last_frame、_last_image 和 _last_right_image 变量。
  16. 循回到第1步，继续处理下一个 tracking_data。
 */


void MapBuilder::TrackingThread(){
  while(!_shutdown){
    if(_tracking_data_buffer.empty()){
      usleep(2000);
      continue;
    }

    TrackingDataPtr tracking_data;
    _tracking_mutex.lock();
    tracking_data = _tracking_data_buffer.front();
    _tracking_data_buffer.pop();
    _tracking_mutex.unlock();

    FramePtr frame = tracking_data->frame;
    FramePtr ref_keyframe = tracking_data->ref_keyframe;
    InputDataPtr input_data = tracking_data->input_data;
    std::vector<cv::DMatch> matches = tracking_data->matches;

    double timestamp = input_data->time;
    cv::Mat image_left_rect = input_data->image_left.clone();
    cv::Mat image_right_rect = input_data->image_right.clone();

    // track
    frame->SetPose(_last_frame->GetPose());
    std::function<int()> track_last_frame = [&](){
      if(_num_since_last_keyframe < 1 || !_last_frame_track_well) return -1;
      InsertKeyframe(_last_frame, _last_right_image);
      _last_keyimage = _last_image;
      matches.clear();
      ref_keyframe = _last_frame;
      return TrackFrame(_last_frame, frame, matches);
    };

    int num_match = matches.size();
    if(num_match < _configs.keyframe_config.min_num_match){
      num_match = track_last_frame();
    }else{
      num_match = TrackFrame(ref_keyframe, frame, matches);
      if(num_match < _configs.keyframe_config.min_num_match){
        num_match = track_last_frame();
      }
    }
    PublishFrame(frame, image_left_rect);

    _last_frame_track_well = (num_match >= _configs.keyframe_config.min_num_match);
    if(!_last_frame_track_well) continue;

    frame->SetPreviousFrame(ref_keyframe);
    _last_frame_track_well = true;

    // for debug 
    // SaveTrackingResult(_last_keyimage, image_left, _last_keyframe, frame, matches, _configs.saving_dir);

    if(AddKeyframe(ref_keyframe, frame, num_match) && ref_keyframe->GetFrameId() == _last_keyframe->GetFrameId()){
      InsertKeyframe(frame, image_right_rect);
      _last_keyimage = image_left_rect;
    }

    _last_frame = frame;
    _last_image = image_left_rect;
    _last_right_image = image_right_rect;
  }  
}

/**
 * 从图像中提取特征点和线段
 * @param image 包含特征的输入图像
 * @param points 存储提取出的特征点的矩阵
 * @param lines 存储提取出的线段的向量
 * @details 函数的逻辑如下：

定义了一个名为extract_point的lambda函数，这个函数使用了外部变量image和points。这个函数首先对一个名为_gpu_mutex的互斥量进行上锁，
然后调用_superpoint对象的infer函数来提取图像image中的点特征，并将结果存储在points中。然后解锁互斥量。如果infer函数执行失败，
则在终端输出一条错误信息。
定义了一个名为extract_line的lambda函数，这个函数使用了外部变量image和lines。这个函数调用_line_detector对象的LineExtractor函数
来提取图像image中的线特征，并将结果存储在lines中。
创建了两个线程point_ectraction_thread和line_ectraction_thread，分别使用extract_point和extract_line函数作为线程函数。
主线程通过调用join函数等待point_ectraction_thread和line_ectraction_thread线程执行完毕，然后继续执行后面的代码。

 */


void MapBuilder::ExtractFeatrue(const cv::Mat& image, Eigen::Matrix<double, 259, Eigen::Dynamic>& points, 
    std::vector<Eigen::Vector4d>& lines){
  std::function<void()> extract_point = [&](){
    _gpu_mutex.lock();
    bool good_infer = _superpoint->infer(image, points);
    _gpu_mutex.unlock();
    if(!good_infer){
      std::cout << "Failed when extracting point features !" << std::endl;
      return;
    }
  };

  std::function<void()> extract_line = [&](){
    _line_detector->LineExtractor(image, lines);
  };

  std::thread point_ectraction_thread(extract_point);
  std::thread line_ectraction_thread(extract_line);

  point_ectraction_thread.join();
  line_ectraction_thread.join();
}


/**
 * 提取图像特征并进行匹配。
 * 
 * @param image 输入影像
 * @param points0 输入的一组点
 * @param points1 用于存储提取的特征点
 * @param lines 用于存储提取的线特征
 * @param matches 用于存储匹配结果
 * @details
 * 这段代码的逻辑如下：

1. 定义了一个函数`extractFeatureAndMatch`，接受一个`image`图像和两个矩阵`points0`和`points1`以及两个向量`lines`和`matches`作为参数。
2. 在函数内部定义了一个lambda函数`extract_point_and_match`，该函数用于提取图像中的特征点，并进行匹配。
3. 在`extract_point_and_match`函数内部，首先通过调用`_superpoint->infer(image, points1)`函数来提取特征点，并将结果存储在`points1`中。
4. 然后通过调用`_point_matching->MatchingPoints(points0, points1, matches)`函数对`points0`和`points1`进行匹配，并将结果存储在`matches`中。
5. 在`extract_point_and_match`函数最后，解锁一个互斥量`_gpu_mutex`，记录特征点提取和匹配的时间。
6. 在函数内部定义了另一个lambda函数`extract_line`，该函数用于提取图像中的线段。
7. 在`extract_line`函数内部，通过调用`_line_detector->LineExtractor(image, lines)`函数提取图像中的线段，并将结果存储在`lines`中。
8. 在`extract_line`函数最后，记录线段提取的时间。
9. 在函数内部，记录整个特征提取和匹配的时间，并输出结果。
10. 创建两个线程，分别执行`extract_point_and_match`和`extract_line`函数。
11. 等待两线程执行完毕。
12. 记录整个特征提取和匹配的时间，并输出结果。
 */

void MapBuilder::ExtractFeatureAndMatch(const cv::Mat& image, const Eigen::Matrix<double, 259, Eigen::Dynamic>& points0, 
    Eigen::Matrix<double, 259, Eigen::Dynamic>& points1, std::vector<Eigen::Vector4d>& lines, std::vector<cv::DMatch>& matches){
  // 使用匿名函数处理点特征的提取和匹配
  std::function<void()> extract_point_and_match = [&](){
    auto point0 = std::chrono::steady_clock::now();
     _gpu_mutex.lock();
     // 如果特征点提取失败，解锁并返回
    if(!_superpoint->infer(image, points1)){
      _gpu_mutex.unlock();
      std::cout << "Failed when extracting point features !" << std::endl;
      return;
    }
    auto point1 = std::chrono::steady_clock::now();

    matches.clear();
        // 匹配点特征

    _point_matching->MatchingPoints(points0, points1, matches);
    _gpu_mutex.unlock();
        // 计算点特征提取和匹配的时间

    auto point2 = std::chrono::steady_clock::now();
    auto point_time = std::chrono::duration_cast<std::chrono::milliseconds>(point1 - point0).count();
    auto point_match_time = std::chrono::duration_cast<std::chrono::milliseconds>(point2 - point1).count();
    // std::cout << "One Frame point Time: " << point_time << " ms." << std::endl;
    // std::cout << "One Frame point match Time: " << point_match_time << " ms." << std::endl;
  };
  // 使用匿名函数处理线特征的提取

  std::function<void()> extract_line = [&](){
    auto line1 = std::chrono::steady_clock::now();
    _line_detector->LineExtractor(image, lines);
    auto line2 = std::chrono::steady_clock::now();
        // 计算线特征提取的时间
    auto line_time = std::chrono::duration_cast<std::chrono::milliseconds>(line2 - line1).count();
    // std::cout << "One Frame line Time: " << line_time << " ms." << std::endl;
  };

  auto feature1 = std::chrono::steady_clock::now();
  std::thread point_ectraction_thread(extract_point_and_match);
  std::thread line_ectraction_thread(extract_line);
  // 并行处理点特征和线特征的提取
  point_ectraction_thread.join();
  line_ectraction_thread.join();

  // 计算特征提取的总时间
  auto feature2 = std::chrono::steady_clock::now();
  auto feature_time = std::chrono::duration_cast<std::chrono::milliseconds>(feature2 - feature1).count();
  // std::cout << "One Frame featrue Time: " << feature_time << " ms." << std::endl;
}

bool MapBuilder::Init(FramePtr frame, cv::Mat& image_left, cv::Mat& image_right){
  // extract features
  Eigen::Matrix<double, 259, Eigen::Dynamic> features_left, features_right;
  std::vector<Eigen::Vector4d> lines_left, lines_right;
  std::vector<cv::DMatch> stereo_matches;
  ExtractFeatrue(image_left, features_left, lines_left);
  int feature_num = features_left.cols();
  if(feature_num < 150) return false;
  ExtractFeatureAndMatch(image_right, features_left, features_right, lines_right, stereo_matches);
  frame->AddLeftFeatures(features_left, lines_left);
  int stereo_point_match = frame->AddRightFeatures(features_right, lines_right, stereo_matches);
  if(stereo_point_match < 100) return false;

  // Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d init_pose;
  init_pose << 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 1, 0, 0, 0, 1;
  frame->SetPose(init_pose);
  frame->SetPoseFixed(true);

  Eigen::Matrix3d Rwc = init_pose.block<3, 3>(0, 0);
  Eigen::Vector3d twc = init_pose.block<3, 1>(0, 3);
  // construct mappoints
  int stereo_point_num = 0;
  std::vector<int> track_ids(feature_num, -1);
  int frame_id = frame->GetFrameId();
  Eigen::Vector3d tmp_position;
  std::vector<MappointPtr> new_mappoints;
  for(size_t i = 0; i < feature_num; i++){
    if(frame->BackProjectPoint(i, tmp_position)){
      tmp_position = Rwc * tmp_position + twc;
      stereo_point_num++;
      track_ids[i] = _track_id++;
      Eigen::Matrix<double, 256, 1> descriptor;
      if(!frame->GetDescriptor(i, descriptor)) continue;
      MappointPtr mappoint = std::shared_ptr<Mappoint>(new Mappoint(track_ids[i], tmp_position, descriptor));
      mappoint->AddObverser(frame_id, i);
      frame->InsertMappoint(i, mappoint);
      new_mappoints.push_back(mappoint);
    }
  }
  frame->SetTrackIds(track_ids);
  if(stereo_point_num < 100) return false;

  // construct maplines
  size_t line_num = frame->LineNum();
  std::vector<MaplinePtr> new_maplines;
  for(size_t i = 0; i < line_num; i++){
    frame->SetLineTrackId(i, _line_track_id);
    MaplinePtr mapline = std::shared_ptr<Mapline>(new Mapline(_line_track_id));
    Vector6d endpoints;
    if(frame->TriangulateStereoLine(i, endpoints)){
      mapline->SetEndpoints(endpoints);
      mapline->SetObverserEndpointStatus(frame_id, 1);
    }else{
      mapline->SetObverserEndpointStatus(frame_id, 0);
    }
    mapline->AddObverser(frame_id, i);
    frame->InsertMapline(i, mapline);
    new_maplines.push_back(mapline);
    _line_track_id++;
  }

  // add frame and mappoints to map
  InsertKeyframe(frame);
  for(MappointPtr mappoint : new_mappoints){
    _map->InsertMappoint(mappoint);
  }
  for(MaplinePtr mapline : new_maplines){
    _map->InsertMapline(mapline);
  }
  _ref_keyframe = frame;
  _last_frame = frame;
  return true;
}

/**
 * 对两个帧中的特征进行匹配，优化帧的位姿，更新特征的追踪ID。
 *
 * @param frame0 第一个帧
 * @param frame1 第二个帧
 * @param matches 匹配结果
 * @return 内点数量
 */
int MapBuilder::TrackFrame(FramePtr frame0, FramePtr frame1, std::vector<cv::DMatch>& matches){
  // 对线特征进行追踪
  Eigen::Matrix<double, 259, Eigen::Dynamic>& features0 = frame0->GetAllFeatures();
  Eigen::Matrix<double, 259, Eigen::Dynamic>& features1 = frame1->GetAllFeatures();
  std::vector<std::map<int, double>> points_on_lines0 = frame0->GetPointsOnLines();
  std::vector<std::map<int, double>> points_on_lines1 = frame1->GetPointsOnLines();
  std::vector<int> line_matches;
  MatchLines(points_on_lines0, points_on_lines1, matches, features0.cols(), features1.cols(), line_matches);

  // 初始化内点标记和匹配点集
  std::vector<int> inliers(frame1->FeatureNum(), -1);
  std::vector<MappointPtr> matched_mappoints(features1.cols(), nullptr);
  std::vector<MappointPtr>& frame0_mappoints = frame0->GetAllMappoints();

  // 更新匹配点集和内点标记
  for(auto& match : matches){
    int idx0 = match.queryIdx;
    int idx1 = match.trainIdx;
    matched_mappoints[idx1] = frame0_mappoints[idx0];
    inliers[idx1] = frame0->GetTrackId(idx0);
  }

  // 通过位姿优化更新内点数量
  int num_inliers = FramePoseOptimization(frame1, matched_mappoints, inliers);

  // 如果内点数量大于配置的最小匹配数，更新追踪ID
  int RM = 0;
  if(num_inliers > _configs.keyframe_config.min_num_match){
    for(std::vector<cv::DMatch>::iterator it = matches.begin(); it != matches.end();){
      int idx0 = (*it).queryIdx;
      int idx1 = (*it).trainIdx;
      if(inliers[idx1] > 0){
        frame1->SetTrackId(idx1, frame0->GetTrackId(idx0));
        frame1->InsertMappoint(idx1, frame0_mappoints[idx0]);
      }
      // 删除外点匹配
      if(inliers[idx1] > 0){
        it++;
      }else{
        it = matches.erase(it);
        RM++;
      }
    }
  }

  // 更新线特征的追踪ID
  const std::vector<MaplinePtr>& frame0_maplines = frame0->GetConstAllMaplines();
  for(size_t i = 0; i < frame0_maplines.size(); i++){
    int j = line_matches[i];
    if(j < 0) continue;
    int line_track_id = frame0->GetLineTrackId(i);

    if(line_track_id >= 0){
      frame1->SetLineTrackId(j, line_track_id);
      frame1->InsertMapline(j, frame0_maplines[i]);
    }
  }

  return num_inliers;
}

/**
 * 对给定帧进行位姿优化的函数
 *
 * @param frame 待优化的帧
 * @param mappoints 地图点的集合
 * @param inliers 内点标记，用于记录优化后的内点
 * @param pose_init 位姿初始化
 * @return 优化后的内点数量
 */
int MapBuilder::FramePoseOptimization(
    FramePtr frame, std::vector<MappointPtr>& mappoints, std::vector<int>& inliers, int pose_init){
  // 使用OpenCV的SolvePnP方法获取初始位姿
  Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
  std::vector<int> cv_inliers;
  int num_cv_inliers = SolvePnPWithCV(frame, mappoints, Twc, cv_inliers);
  
  // 检查位姿差异，如果差异过大或内点数量过少则使用上一帧位姿作为当前帧位姿
  Eigen::Vector3d check_dp = Twc.block<3, 1>(0, 3) - _last_frame->GetPose().block<3, 1>(0, 3);
  if(check_dp.norm() > 0.5 || num_cv_inliers < _configs.keyframe_config.min_num_match){
    Twc = _last_frame->GetPose();
  }

  // 定义优化所需的数据结构
  MapOfPoses poses;
  MapOfPoints3d points;
  std::vector<CameraPtr> camera_list;
  VectorOfMonoPointConstraints mono_point_constraints;
  VectorOfStereoPointConstraints stereo_point_constraints;

  camera_list.emplace_back(_camera);

  // 初始化位
  Pose3d pose;
  pose.p = Twc.block<3, 1>(0, 3);
  pose.q = Twc.block<3, 3>(0, 0);
  int frame_id = frame->GetFrameId();    
  poses.insert(std::pair<int, Pose3d>(frame_id, pose));  

  // 按照地图点进行视觉约束的构造
  std::vector<size_t> mono_indexes;
  std::vector<size_t> stereo_indexes;

  for(size_t i = 0; i < mappoints.size(); i++){
  // 遍历地图点，生成视觉约束
    MappointPtr mpt = mappoints[i];
    // 如果地图点存在并有效，生成对应点的数据并加入points
    if(mpt == nullptr || !mpt->IsValid()) continue;
    Eigen::Vector3d keypoint; 
    if(!frame->GetKeypointPosition(i, keypoint)) continue;

    int mpt_id = mpt->GetId();
    Position3d point;
    point.p = mpt->GetPosition();
    point.fixed = true;
    points.insert(std::pair<int, Position3d>(mpt_id, point));

    // 根据地图点的深度信息生成对应的视觉约束
    if(keypoint(2) > 0){
      // 生成立体视觉约束
      StereoPointConstraintPtr stereo_constraint = std::shared_ptr<StereoPointConstraint>(new StereoPointConstraint()); 
      stereo_constraint->id_pose = frame_id;
      stereo_constraint->id_point = mpt_id;
      stereo_constraint->id_camera = 0;
      stereo_constraint->inlier = true;
      stereo_constraint->keypoint = keypoint;
      stereo_constraint->pixel_sigma = 0.8;
      stereo_point_constraints.push_back(stereo_constraint);
      stereo_indexes.push_back(i);
    }else{
      // 生成单目视觉约束
      MonoPointConstraintPtr mono_constraint = std::shared_ptr<MonoPointConstraint>(new MonoPointConstraint()); 
      mono_constraint->id_pose = frame_id;
      mono_constraint->id_point = mpt_id;
      mono_constraint->id_camera = 0;
      mono_constraint->inlier = true;
      mono_constraint->keypoint = keypoint.head(2);
      mono_constraint->pixel_sigma = 0.8;
      mono_point_constraints.push_back(mono_constraint);
      mono_indexes.push_back(i);
    }

  }
  
  // 执行优化
  int num_inliers = FrameOptimization(poses, points, camera_list, mono_point_constraints, 
      stereo_point_constraints, _configs.tracking_optimization_config);

  // 如果优化后的内点数量足够多，则更新帧位姿以及内点标记
  if(num_inliers > _configs.keyframe_config.min_num_match){
    // 更新帧位姿
    Eigen::Matrix4d frame_pose = Eigen::Matrix4d::Identity();
    frame_pose.block<3, 3>(0, 0) = poses.begin()->second.q.matrix();
    frame_pose.block<3, 1>(0, 3) = poses.begin()->second.p;
    frame->SetPose(frame_pose);

    // 更新内点标记
    for(size_t i = 0; i < mono_point_constraints.size(); i++){
      size_t idx = mono_indexes[i];
      if(!mono_point_constraints[i]->inlier){
        inliers[idx] = -1;
      }
    }

    for(size_t i = 0; i < stereo_point_constraints.size(); i++){
      size_t idx = stereo_indexes[i];
      if(!stereo_point_constraints[i]->inlier){
        inliers[idx] = -1;
      }
    }

  }

  return num_inliers;
}


/**
 * 判断当前帧是否能作为关键帧
 * @param last_keyframe 上一关键帧
 * @param current_frame 当前帧
 * @param num_match 匹配的特征点数量
 * @return 返回是否可以作为关键帧，如果满足以下任一条件，返回true：匹配的特征点数量不足；当前帧与前一关键帧之间的旋转角度过大；当前帧与前一关键帧之间的位移距离过大；从上一关键帧到当前帧的帧数过多
 */
bool MapBuilder::AddKeyframe(FramePtr last_keyframe, FramePtr current_frame, int num_match){
  // 获取当前帧的位姿
  Eigen::Matrix4d frame_pose = current_frame->GetPose();
  // 获取上一关键帧的位姿
  Eigen::Matrix4d& last_keyframe_pose = _last_keyframe->GetPose();
  // 提取旋转矩阵和平移向量
  Eigen::Matrix3d last_R = last_keyframe_pose.block<3, 3>(0, 0);
  Eigen::Vector3d last_t = last_keyframe_pose.block<3, 1>(0, 3);
  Eigen::Matrix3d current_R = frame_pose.block<3, 3>(0, 0);
  Eigen::Vector3d current_t = frame_pose.block<3, 1>(0, 3);

  // 计算上一关键帧和当前帧之间的旋转差和位移差
  Eigen::Matrix3d delta_R = last_R.transpose() * current_R;
  Eigen::AngleAxisd angle_axis(delta_R); 
  double delta_angle = angle_axis.angle();
  double delta_distance = (current_t - last_t).norm();

  // 计算从上一关键帧到当前帧的帧数
  int passed_frame_num = current_frame->GetFrameId() - _last_keyframe->GetFrameId();

  // 判断是否满足作为关键帧的条件
  bool not_enough_match = (num_match < _configs.keyframe_config.max_num_match);
  bool large_delta_angle = (delta_angle > _configs.keyframe_config.max_angle);
  bool large_distance = (delta_distance > _configs.keyframe_config.max_distance);
  bool enough_passed_frame = (passed_frame_num > _configs.keyframe_config.max_num_passed_frame);
  return (not_enough_match || large_delta_angle || large_distance || enough_passed_frame);
}

void MapBuilder::InsertKeyframe(FramePtr frame, const cv::Mat& image_right){
  _last_keyframe = frame;

  Eigen::Matrix<double, 259, Eigen::Dynamic> features_right;
  std::vector<Eigen::Vector4d> lines_right;
  std::vector<cv::DMatch> stereo_matches;

  ExtractFeatureAndMatch(image_right, frame->GetAllFeatures(), features_right, lines_right, stereo_matches);
  frame->AddRightFeatures(features_right, lines_right, stereo_matches);
  InsertKeyframe(frame);
}

void MapBuilder::InsertKeyframe(FramePtr frame){
  _last_keyframe = frame;

  // create new track id
  std::vector<int>& track_ids = frame->GetAllTrackIds();
  for(size_t i = 0; i < track_ids.size(); i++){
    if(track_ids[i] < 0){
      frame->SetTrackId(i, _track_id++);
    }
  }

  // create new line track id
  const std::vector<int>& line_track_ids = frame->GetAllLineTrackId();
  for(size_t i = 0; i < line_track_ids.size(); i++){
    if(line_track_ids[i] < 0){
      frame->SetLineTrackId(i, _line_track_id++);
    }
  }

  // insert keyframe to map
  _map->InsertKeyframe(frame);

  // update last keyframe
  _num_since_last_keyframe = 1;
  _ref_keyframe = frame;
  _to_update_local_map = true;
}

void MapBuilder::UpdateReferenceFrame(FramePtr frame){
  int current_frame_id = frame->GetFrameId();
  std::vector<MappointPtr>& mappoints = frame->GetAllMappoints();
  std::map<FramePtr, int> keyframes;
  for(MappointPtr mpt : mappoints){
    if(!mpt || mpt->IsBad()) continue;
    const std::map<int, int> obversers = mpt->GetAllObversers();
    for(auto& kv : obversers){
      int observer_id = kv.first;
      if(observer_id == current_frame_id) continue;
      FramePtr keyframe = _map->GetFramePtr(observer_id);
      if(!keyframe) continue;
      keyframes[keyframe]++;
    }
  }
  if(keyframes.empty()) return;

  std::pair<FramePtr, int> max_covi = std::pair<FramePtr, int>(nullptr, -1);
  for(auto& kv : keyframes){
    if(kv.second > max_covi.second){
      max_covi = kv;
    }
  }
 
  if(max_covi.first->GetFrameId() != _ref_keyframe->GetFrameId()){
    _ref_keyframe = max_covi.first;
    _to_update_local_map = true;
  }
}

void MapBuilder::UpdateLocalKeyframes(FramePtr frame){
  _local_keyframes.clear();
  std::vector<std::pair<int, FramePtr>> neighbor_frames = _ref_keyframe->GetOrderedConnections(-1);
  for(auto& kv : neighbor_frames){
    _local_keyframes.push_back(kv.second);
  }
}

void MapBuilder::UpdateLocalMappoints(FramePtr frame){
  _local_mappoints.clear();
  int current_frame_id = frame->GetFrameId();
  for(auto& kf : _local_keyframes){
    const std::vector<MappointPtr>& mpts = kf->GetAllMappoints();
    for(auto& mpt : mpts){
      if(mpt && mpt->IsValid() && mpt->tracking_frame_id != current_frame_id){
        mpt->tracking_frame_id = current_frame_id;
        _local_mappoints.push_back(mpt);
      }
    }
  }
}

void MapBuilder::SearchLocalPoints(FramePtr frame, std::vector<std::pair<int, MappointPtr>>& good_projections){
  int current_frame_id = frame->GetFrameId();
  std::vector<MappointPtr>& mpts = frame->GetAllMappoints();
  for(auto& mpt : mpts){
    if(mpt && !mpt->IsBad()) mpt->last_frame_seen = current_frame_id;
  }

  std::vector<MappointPtr> selected_mappoints;
  for(auto& mpt : _local_mappoints){
    if(mpt && mpt->IsValid() && mpt->last_frame_seen != current_frame_id){
      selected_mappoints.push_back(mpt);
    }
  }

  _map->SearchByProjection(frame, selected_mappoints, 1, good_projections);
}

int MapBuilder::TrackLocalMap(FramePtr frame, int num_inlier_thr){
  if(_to_update_local_map){
    UpdateLocalKeyframes(frame);
    UpdateLocalMappoints(frame);
  }

  std::vector<std::pair<int, MappointPtr>> good_projections;
  SearchLocalPoints(frame, good_projections);
  if(good_projections.size() < 3) return -1;

  std::vector<MappointPtr> mappoints = frame->GetAllMappoints();
  for(auto& good_projection : good_projections){
    int idx = good_projection.first;
    if(mappoints[idx] && !mappoints[idx]->IsBad()) continue;
    mappoints[idx] = good_projection.second;
  }

  std::vector<int> inliers(mappoints.size(), -1);
  int num_inliers = FramePoseOptimization(frame, mappoints, inliers, 2);

  // update track id
  if(num_inliers > _configs.keyframe_config.min_num_match && num_inliers > num_inlier_thr){
    for(size_t i = 0; i < mappoints.size(); i++){
      if(inliers[i] > 0){
        frame->SetTrackId(i, mappoints[i]->GetId());
        frame->InsertMappoint(i, mappoints[i]);
      }
    }
  }else{
    num_inliers = -1;
  }
  return num_inliers;
}

void MapBuilder::PublishFrame(FramePtr frame, cv::Mat& image){
  FeatureMessgaePtr feature_message = std::shared_ptr<FeatureMessgae>(new FeatureMessgae);
  FramePoseMessagePtr frame_pose_message = std::shared_ptr<FramePoseMessage>(new FramePoseMessage);

  feature_message->time = frame->GetTimestamp();
  feature_message->image = image;
  feature_message->keypoints = frame->GetAllKeypoints();;
  feature_message->lines = frame->GatAllLines();
  feature_message->points_on_lines = frame->GetPointsOnLines();
  std::vector<bool> inliers_feature_message;
  frame->GetInlierFlag(inliers_feature_message);
  feature_message->inliers = inliers_feature_message;
  frame_pose_message->time = frame->GetTimestamp();
  frame_pose_message->pose = frame->GetPose();
  feature_message->line_track_ids = frame->GetAllLineTrackId();

  _ros_publisher->PublishFeature(feature_message);
  _ros_publisher->PublishFramePose(frame_pose_message);
}

void MapBuilder::SaveTrajectory(){
  std::string file_path = ConcatenateFolderAndFileName(_configs.saving_dir, "keyframe_trajectory.txt");
  _map->SaveKeyframeTrajectory(file_path);
}

void MapBuilder::SaveTrajectory(std::string file_path){
  _map->SaveKeyframeTrajectory(file_path);
}

void MapBuilder::SaveMap(const std::string& map_root){
  _map->SaveMap(map_root);
}

void MapBuilder::ShutDown(){
  _shutdown = true;
  _feature_thread.join();
  _tracking_thread.join();
}
