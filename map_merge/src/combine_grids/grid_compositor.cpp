/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
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

#include <combine_grids/grid_compositor.h>

#include <opencv2/stitching/detail/util.hpp>

#include <ros/assert.h>

namespace combine_grids
{
namespace internal
{
nav_msgs::OccupancyGrid::Ptr GridCompositor::compose(
  //grids：所有映射后的数据，rois：所有映射后roi
    const std::vector<cv::Mat>& grids, const std::vector<cv::Rect>& rois)
{
  ROS_ASSERT(grids.size() == rois.size());
  //只有一张map,包含所有数据
  nav_msgs::OccupancyGrid::Ptr result_grid(new nav_msgs::OccupancyGrid());
 //角点和size,其实就是把roi分出来
 //corners表示左上角坐标数组
 //grids-height,width,value
  std::vector<cv::Point> corners;
  corners.reserve(grids.size());
  std::vector<cv::Size> sizes;
  sizes.reserve(grids.size());
  //输入所有图像roi的左上角和size-x,y,height,width
  for (auto& roi : rois) {
    corners.push_back(roi.tl());
    sizes.push_back(roi.size());
  }
  //计算出全景图大小
  cv::Rect dst_roi = cv::detail::resultRoi(corners, sizes);

  result_grid->info.width = static_cast<uint>(dst_roi.width);
  result_grid->info.height = static_cast<uint>(dst_roi.height);
  result_grid->data.resize(static_cast<size_t>(dst_roi.area()), -1);
  // create view for opencv pointing to newly allocated grid
  //这里result_grid包含数据,并将result的所有数值初始化为result_grid
  //注意这里的构造函数，是共享内存的，所以下面看似是在修改别的，实际上自身也发生了变化
  cv::Mat result(dst_roi.size(), CV_8S, result_grid->data.data());

  
  for (size_t i = 0; i < grids.size(); ++i) {
    // we need to compensate global offset
    //每一个roi
    cv::Rect roi = cv::Rect(corners[i] - dst_roi.tl(), sizes[i]);
    //转换后的map一张张贴进去，访问result的roi区域
    cv::Mat result_roi(result, roi);
    // reinterpret warped matrix as signed
    // we will not change this matrix, but opencv does not support const matrices
    //把grids的value进行填充（0,-1,1），其实就是images_
    cv::Mat warped_signed (grids[i].size(), CV_8S, const_cast<uchar*>(grids[i].ptr()));
    // compose img into result matrix，不断选最大的
    cv::max(result_roi, warped_signed, result_roi);
  }

  return result_grid;
}

}  // namespace internal

}  // namespace combine_grids
