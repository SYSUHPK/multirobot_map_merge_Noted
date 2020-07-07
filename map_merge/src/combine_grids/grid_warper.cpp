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

#include <combine_grids/grid_warper.h>

#include <opencv2/stitching/detail/warpers.hpp>

#include <ros/assert.h>


//在得到相机的相对位置，如果直接进行拼接会破坏视场的一致性，使得拼接得到的全景图看起来不够连贯，因此需要通过投影变换
//相对位置是在executeposeEstimation中实现的
namespace combine_grids
{
namespace internal
{
  //有点像裁剪大小，分类，grid-images_，roi其实只有感兴趣的区域-corner,height,width
cv::Rect GridWarper::warp(const cv::Mat& grid, const cv::Mat& transform,
                          cv::Mat& warped_grid)
{
  ROS_ASSERT(transform.type() == CV_64F);
  cv::Mat H;
  //求逆矩阵
  invertAffineTransform(transform.rowRange(0, 2), H);
  //先映射roi-rect(x, y, width, height)，再映射grid到warped_grid-width,height,value
  cv::Rect roi = warpRoi(grid, H);
  // shift top left corner for warp affine (otherwise the image is cropped)
  H.at<double>(0, 2) -= roi.tl().x;
  H.at<double>(1, 2) -= roi.tl().y;
  //仿射变换-grid 输入,其实是images_；warped_grid 输出；H 变换矩阵,单应矩阵，计算图片之间的变化
  //这时候warped_grid-heigth.width,0,-1,1
  warpAffine(grid, warped_grid, H, roi.size(), cv::INTER_NEAREST,
             cv::BORDER_CONSTANT,
             cv::Scalar::all(255) /* this is -1 for signed char */);//border value
  ROS_ASSERT(roi.size() == warped_grid.size());

  return roi;
}

cv::Rect GridWarper::warpRoi(const cv::Mat& grid, const cv::Mat& transform)
{
  //返回的应该是roi
  cv::Ptr<cv::detail::PlaneWarper> warper =
      cv::makePtr<cv::detail::PlaneWarper>();
  cv::Mat H;
  //格式
  transform.convertTo(H, CV_32F);

  // separate rotation and translation for plane warper
  // 3D translation
  //把transform的Mat格式CV_64F转化为CV_32F，存入H，然后分为R和T
  cv::Mat T = cv::Mat::zeros(3, 1, CV_32F);
  H.colRange(2, 3).rowRange(0, 2).copyTo(T.rowRange(0, 2));
  // 3D rotation
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  H.colRange(0, 2).copyTo(R.rowRange(0, 2).colRange(0, 2));

  return warper->warpRoi(grid.size(), cv::Mat::eye(3, 3, CV_32F), R, T);
}

}  // namespace internal

}  // namespace combine_grids
