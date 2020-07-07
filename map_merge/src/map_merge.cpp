/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <thread>

#include <map_merge/map_merge.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace map_merge
{
MapMerge::MapMerge() : subscriptions_size_(0)
{
  ros::NodeHandle private_nh("~");
  std::string frame_id;
  std::string merged_map_topic;
  //parameters setting 参数设置
  private_nh.param("merging_rate", merging_rate_, 4.0);
  private_nh.param("discovery_rate", discovery_rate_, 0.05);
  private_nh.param("estimation_rate", estimation_rate_, 0.5);
  private_nh.param("known_init_poses", have_initial_poses_, true);
  private_nh.param("estimation_confidence", confidence_threshold_, 1.0);
  private_nh.param<std::string>("robot_map_topic", robot_map_topic_, "map");
  private_nh.param<std::string>("robot_map_updates_topic",
                                robot_map_updates_topic_, "map_updates");
  private_nh.param<std::string>("robot_namespace", robot_namespace_, "");
  private_nh.param<std::string>("merged_map_topic", merged_map_topic, "map");
  private_nh.param<std::string>("world_frame", world_frame_, "world");

  /* publishing */
  merged_map_publisher_ =
      node_.advertise<nav_msgs::OccupancyGrid>(merged_map_topic, 50, true);
}

/*
 * Subcribe to pose and map topics
 */
void MapMerge::topicSubscribing()
{
  ROS_DEBUG("Robot discovery started.");
  // get topic infomation from master, like topiclist 
  //从master的topic信息中获取topic，类似topiclist
  ros::master::V_TopicInfo topic_infos;
  geometry_msgs::Transform init_pose;
  std::string robot_name;
  std::string map_topic;
  std::string map_updates_topic;

  ros::master::getTopics(topic_infos);
  // default msg constructor does no properly initialize quaternion
  init_pose.rotation.w = 1;  // create identity quaternion

  for (const auto& topic : topic_infos) {
    // we check only map topic
    //遍历订阅的topics，只检测map topic
    if (!isRobotMapTopic(topic)) {
      continue;
    }
    //robots_: maps robots namespaces to maps
    robot_name = robotNameFromTopic(topic.name);
    if (robots_.count(robot_name)) {
      // we already know this robot
      continue;
    }
    //判断是否有初始化位姿，其实就是位姿，一般都是有的
    if (have_initial_poses_ && !getInitPose(robot_name, init_pose)) {
      ROS_WARN("Couldn't get initial position for robot [%s]\n"
               "did you defined parameters map_merge/init_pose_[xyz]? in robot "
               "namespace? If you want to run merging without known initial "
               "positions of robots please set `known_init_poses` parameter "
               "to false. See relavant documentation for details.",
               robot_name.c_str());
      continue;
    }
    //用lock_guard锁，在list类型的subscriptions_中不断插空，list包含几乎所有数据-geometry_msgs::Transform initial_pose; nav_msgs::OccupancyGrid
    ROS_INFO("adding robot [%s] to system", robot_name.c_str());
    {
      std::lock_guard<boost::shared_mutex> lock(subscriptions_mutex_);
      subscriptions_.emplace_front();
      ++subscriptions_size_;
    }
    //解锁

    // no locking here. robots_ are used only in this procedure
    //不断取前面的一位，插入新的
    MapSubscription& subscription = subscriptions_.front();
    robots_.insert({robot_name, &subscription});
    subscription.initial_pose = init_pose;

    /* subscribe callbacks */
    //命名空间+topic，类似<robot_namespace>/map (nav_msgs/OccupancyGrid)，
    //这里应该是一次循环订阅一个机器人的主题，耗费不了多久，其实里面调用进行了读取数据，所以是有先后的区别，
    //然后不同topic的的数据会顺序轮流存储在subscription中，subscriptions中的每一个单元，
    //一个存储一种，轮换，所以其实是顺序轮流存储
    map_topic = ros::names::append(robot_name, robot_map_topic_);
    map_updates_topic =
        ros::names::append(robot_name, robot_map_updates_topic_);
    ROS_INFO("Subscribing to MAP topic: %s.", map_topic.c_str());
    subscription.map_sub = node_.subscribe<nav_msgs::OccupancyGrid>(
        map_topic, 50,
        [this, &subscription](const nav_msgs::OccupancyGrid::ConstPtr& msg) {
          fullMapUpdate(msg, subscription);
        });
    ROS_INFO("Subscribing to MAP updates topic: %s.",
             map_updates_topic.c_str());
    subscription.map_updates_sub =
        node_.subscribe<map_msgs::OccupancyGridUpdate>(
            map_updates_topic, 50,
            [this, &subscription](
                const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {
              partialMapUpdate(msg, subscription);
            });
  }
}

/*
 * mapMerging()
 */
//注意，这里并不是建图，而是融合，如果初始位姿知道了，那就不需要，看数据类型，好像也是两个坐标系之间的变换
void MapMerge::mapMerging()
{
  ROS_DEBUG("Map merging started.");

  if (have_initial_poses_) {
    //共享锁，subscriptions在grids和transform中插入数据,防止数据更新之类的
    //这个在poseEstimation也有
    std::vector<nav_msgs::OccupancyGridConstPtr> grids;
    std::vector<geometry_msgs::Transform> transforms;
    grids.reserve(subscriptions_size_);
    {
      boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);
      for (auto& subscription : subscriptions_) {
        std::lock_guard<std::mutex> s_lock(subscription.mutex);
        grids.push_back(subscription.readonly_map);
        transforms.push_back(subscription.initial_pose);
      }
    }
    // we don't need to lock here, because when have_initial_poses_ is true we
    // will not run concurrently on the pipeline
    //pipeline读取grids的readonly数据，以及所有位姿,其实feed是将grids数据转化为image（opencv模式），settransform也是一样的
    //这里有锁，所以即使里面的变量修改了也一样
    pipeline_.feed(grids.begin(), grids.end());
    pipeline_.setTransforms(transforms.begin(), transforms.end());
  }
  //开始merge,composeGrids
  nav_msgs::OccupancyGridPtr merged_map;
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    merged_map = pipeline_.composeGrids();
  }
  if (!merged_map) {
    return;
  }

  //发布merge map的数据，map (nav_msgs/OccupancyGrid)
  ROS_DEBUG("all maps merged, publishing");
  ros::Time now = ros::Time::now();
  merged_map->info.map_load_time = now;
  merged_map->header.stamp = now;
  merged_map->header.frame_id = world_frame_;

  ROS_ASSERT(merged_map->info.resolution > 0.f);
  merged_map_publisher_.publish(merged_map);
}

//地图位姿计算
void MapMerge::poseEstimation()
{
  //grids存储只读地图
  ROS_DEBUG("Grid pose estimation started.");
  std::vector<nav_msgs::OccupancyGridConstPtr> grids;
  grids.reserve(subscriptions_size_);
  {
    boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);
    for (auto& subscription : subscriptions_) {
      std::lock_guard<std::mutex> s_lock(subscription.mutex);
      grids.push_back(subscription.readonly_map);
    }
  }
  //data进入pipeline
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  pipeline_.feed(grids.begin(), grids.end());
  // TODO allow user to change feature type
  //计算Transform
  pipeline_.estimateTransforms(combine_grids::FeatureType::AKAZE,
                               confidence_threshold_);
}
//比较stamp，比较快就更新，注意这里的更新其实是指数据传输到merge的node，并不是返回的更新
void MapMerge::fullMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr& msg,
                             MapSubscription& subscription)
{
  ROS_DEBUG("received full map update");
  std::lock_guard<std::mutex> lock(subscription.mutex);
  if (subscription.readonly_map &&
      subscription.readonly_map->header.stamp > msg->header.stamp) {
    // we have been overrunned by faster update. our work was useless.
    return;
  }
  //在这一步把subscribe的数据存储起来，所以前面的subscribe的函数里面调用了这个之后就可以使用
  subscription.readonly_map = msg;
  subscription.writable_map = nullptr;
}
//
void MapMerge::partialMapUpdate(
    const map_msgs::OccupancyGridUpdate::ConstPtr& msg,
    MapSubscription& subscription)
{
  ROS_DEBUG("received partial map update");

  if (msg->x < 0 || msg->y < 0) {
    ROS_ERROR("negative coordinates, invalid update. x: %d, y: %d", msg->x,
              msg->y);
    return;
  }

  size_t x0 = static_cast<size_t>(msg->x);
  size_t y0 = static_cast<size_t>(msg->y);
  size_t xn = msg->width + x0;
  size_t yn = msg->height + y0;

  nav_msgs::OccupancyGridPtr map;
  nav_msgs::OccupancyGridConstPtr readonly_map;  // local copy
  {
    // load maps
    std::lock_guard<std::mutex> lock(subscription.mutex);
    map = subscription.writable_map;
    readonly_map = subscription.readonly_map;
  }

  if (!readonly_map) {
    ROS_WARN("received partial map update, but don't have any full map to "
             "update. skipping.");
    return;
  }

  // we don't have partial map to take update, we must copy readonly map and
  // update new writable map
  //readonly map更新了，用map更新readonly map
  if (!map) {
    map.reset(new nav_msgs::OccupancyGrid(*readonly_map));
  }

  size_t grid_xn = map->info.width;
  size_t grid_yn = map->info.height;

  if (xn > grid_xn || x0 > grid_xn || yn > grid_yn || y0 > grid_yn) {
    ROS_WARN("received update doesn't fully fit into existing map, "
             "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
             "map is: [0, %lu], [0, %lu]",
             x0, xn, y0, yn, grid_xn, grid_yn);
  }

  // update map with data
  size_t i = 0;
  for (size_t y = y0; y < yn && y < grid_yn; ++y) {
    for (size_t x = x0; x < xn && x < grid_xn; ++x) {
      size_t idx = y * grid_xn + x;  // index to grid for this specified cell
      map->data[idx] = msg->data[i];
      ++i;
    }
  }
  // update time stamp
  map->header.stamp = msg->header.stamp;

  {
    // store back updated map
    std::lock_guard<std::mutex> lock(subscription.mutex);
    if (subscription.readonly_map &&
        subscription.readonly_map->header.stamp > map->header.stamp) {
      // we have been overrunned by faster update. our work was useless.
      return;
    }
    subscription.writable_map = map;
    subscription.readonly_map = map;
  }
}
//获取robot的namespace
std::string MapMerge::robotNameFromTopic(const std::string& topic)
{
  return ros::names::parentNamespace(topic);
}

/* identifies topic via suffix */
//确定筛选所要的topic
bool MapMerge::isRobotMapTopic(const ros::master::TopicInfo& topic)
{
  /* test whether topic is robot_map_topic_ */
  std::string topic_namespace = ros::names::parentNamespace(topic.name); 
  bool is_map_topic =
      ros::names::append(topic_namespace, robot_map_topic_) == topic.name;

  /* test whether topic contains *anywhere* robot namespace */
  //表示不存在的位置，有机器人namespace存在就好
  auto pos = topic.name.find(robot_namespace_); 
  bool contains_robot_namespace = pos != std::string::npos; 

  /* we support only occupancy grids as maps */
  //订阅的只有OccupancyGrid
  bool is_occupancy_grid = topic.datatype == "nav_msgs/OccupancyGrid";

  /* we don't want to subcribe on published merged map */
  //不是发布的topic
  bool is_our_topic = merged_map_publisher_.getTopic() == topic.name;

  return is_occupancy_grid && !is_our_topic && contains_robot_namespace &&
         is_map_topic;
}

/*
 * Get robot's initial position
 */
//获取机器人的初始位姿，其实感觉是
bool MapMerge::getInitPose(const std::string& name,
                           geometry_msgs::Transform& pose)
{
  std::string merging_namespace = ros::names::append(name, "map_merge");
  double yaw = 0.0;

  bool success =
      ros::param::get(ros::names::append(merging_namespace, "init_pose_x"),
                      pose.translation.x) &&
      ros::param::get(ros::names::append(merging_namespace, "init_pose_y"),
                      pose.translation.y) &&
      ros::param::get(ros::names::append(merging_namespace, "init_pose_z"),
                      pose.translation.z) &&
      ros::param::get(ros::names::append(merging_namespace, "init_pose_yaw"),
                      yaw);

  tf2::Quaternion q;
  q.setEuler(0., 0., yaw);
  pose.rotation = toMsg(q);

  return success;
}

/*
 * execute()
 */
void MapMerge::executemapMerging()
{
  ros::Rate r(merging_rate_);
  while (node_.ok()) {
    mapMerging();
    r.sleep();
  }
}

void MapMerge::executetopicSubscribing()
{
  ros::Rate r(discovery_rate_);
  while (node_.ok()) {
    topicSubscribing();
    r.sleep();
  }
}

void MapMerge::executeposeEstimation()
{
  //这个就有点意思，其实就是只有没有初始位姿才需要通过特征点匹配去做，这里不需要，一般都是有的
  if (have_initial_poses_)
    return;

  ros::Rate r(estimation_rate_);
  while (node_.ok()) {
    poseEstimation();
    r.sleep();
  }
}

/*
 * spin()
 */
void MapMerge::spin()
{
  //第一个似乎无所谓
  ros::spinOnce();
  std::thread merging_thr([this]() { executemapMerging(); });
  std::thread subscribing_thr([this]() { executetopicSubscribing(); });
  std::thread estimation_thr([this]() { executeposeEstimation(); });
  ros::spin();
  estimation_thr.join();
  merging_thr.join();
  subscribing_thr.join();
}

}  // namespace map_merge

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_merge");
  // this package is still in development -- start wil debugging enabled
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  map_merge::MapMerge map_merging;
  map_merging.spin();
  return 0;
}
