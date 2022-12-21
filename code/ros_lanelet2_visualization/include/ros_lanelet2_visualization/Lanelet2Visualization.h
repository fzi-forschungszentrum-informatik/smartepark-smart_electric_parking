// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
//Copyright (c) 2022, FZI Forschungszentrum Informatik
//
//Redistribution and use in source and binary forms, with or without modification, are permitted
//provided that the following conditions are met:
//
//1. Redistributions of source code must retain the above copyright notice, this list of conditions
//   and the following disclaimer.
//
//2. Redistributions in binary form must reproduce the above copyright notice, this list of
//   conditions and the following disclaimer in the documentation and/or other materials provided
//   with the distribution.

//3. Neither the name of the copyright holder nor the names of its contributors may be used to
//   endorse or promote products derived from this software without specific prior written
//   permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
//IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
//FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
//CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
//WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Philip Schörner <schoerner@fzi.de>
 * \date    2020-01-23
 *
 */
//----------------------------------------------------------------------
#ifndef ROS_LANELET2_VISUALIZATION_LANELET2_VISUALIZATION_H_INLCUDED
#define ROS_LANELET2_VISUALIZATION_LANELET2_VISUALIZATION_H_INLCUDED

// ROS includes
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/LaneletMap.h>

namespace ros_lanelet2_visualization {

visualization_msgs::Marker
toMarker(lanelet::ConstLineString3d linestring,
         const std::string& frame_id                 = "/map",
         const std::string& ns                       = "lanelet line strip",
         visualization_msgs::Marker::_type_type type = visualization_msgs::Marker::LINE_STRIP,
         double scale                                = 0.05,
         double color_r                              = 1.0,
         double color_g                              = 0.0,
         double color_b                              = 0.0,
         double color_a                              = 1.0,
         double lifetime                             = 0.0)
{
  visualization_msgs::Marker marker;
  marker.header.stamp    = ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.id              = linestring.id();
  marker.ns              = ns;
  marker.type            = type;
  marker.action          = visualization_msgs::Marker::ADD;
  marker.lifetime        = ros::Duration(lifetime);
  marker.frame_locked    = true;

  marker.pose.position.x = 0.;
  marker.pose.position.y = 0.;
  marker.pose.position.z = 0.;
  marker.pose.orientation.w = 1.;
  marker.pose.orientation.x = 0.;
  marker.pose.orientation.y = 0.;
  marker.pose.orientation.z = 0.;

  marker.scale.x = scale; // LINE_STRIP only uses the x component of scale for line width
  if (type != visualization_msgs::Marker::LINE_STRIP && type != visualization_msgs::Marker::LINE_LIST)
  {
    marker.scale.y = scale; // LINE_STRIP only uses the x component of scale for line width
    marker.scale.z = scale; // LINE_STRIP only uses the x component of scale for line width
  }
  marker.color.r = color_r;
  marker.color.g = color_g;
  marker.color.b = color_b;
  marker.color.a = color_a;

  geometry_msgs::Point p_msg;

  for (const auto& p : linestring)
  {
    p_msg.x = p.x();
    p_msg.y = p.y();
    p_msg.z = p.z();
    marker.points.push_back(p_msg);
  }
  return marker;
}


/*! Create a MarkerArray for a \a lanelet by invoking conversion to line strip.
 *  The line stips are aggregated in one namespace per lane containing the lane id.
 */
visualization_msgs::MarkerArray toMarkerArray(lanelet::ConstLanelet lanelet,
                                              const std::string& frame_id         = "/map",
                                              const std::string& namespace_prefix = "",
                                              double scale                        = 0.05,
                                              double color_r                      = 1.0,
                                              double color_g                      = 0.0,
                                              double color_b                      = 0.0,
                                              double color_a                      = 1.0,
                                              double lifetime                     = 0.0)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker strip;

  std::stringstream ss;
  ss << namespace_prefix << "Lanelet " << lanelet.id();
  strip = toMarker(lanelet.leftBound(),
                   frame_id,
                   ss.str(),
                   visualization_msgs::Marker::LINE_STRIP,
                   scale,
                   color_r,
                   color_g,
                   color_b,
                   color_a,
                   lifetime);

  strip.id = 0;
  marker_array.markers.push_back(strip);

  strip = toMarker(lanelet.rightBound(),
                   frame_id,
                   ss.str(),
                   visualization_msgs::Marker::LINE_STRIP,
                   scale,
                   color_r,
                   color_g,
                   color_b,
                   color_a,
                   lifetime);

  strip.id = 1;
  marker_array.markers.push_back(strip);

  strip = toMarker(lanelet.centerline(),
                   frame_id,
                   ss.str(),
                   visualization_msgs::Marker::LINE_STRIP,
                   scale / 3.,
                   color_r,
                   color_g,
                   color_b,
                   color_a,
                   lifetime);

  strip.id = 2;
  marker_array.markers.push_back(strip);

  ss.str("");
  ss << lanelet.id();
  // id
  visualization_msgs::Marker id_marker;
  id_marker.header.frame_id = frame_id;
  id_marker.header.stamp    = ros::Time();
  id_marker.ns              = "ids";
  id_marker.id              = lanelet.id();
  id_marker.text            = ss.str();
  id_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
  id_marker.action          = visualization_msgs::Marker::ADD;
  id_marker.frame_locked    = true;

  id_marker.pose.position.x =
    (lanelet.rightBound().front().x() + lanelet.leftBound().front().x() + lanelet.rightBound().back().x() + lanelet.leftBound().back().x()) / 4.;
  id_marker.pose.position.y =
    (lanelet.rightBound().front().y() + lanelet.leftBound().front().y() + lanelet.rightBound().back().y() + lanelet.leftBound().back().y()) / 4.;
  id_marker.pose.position.z    = 0.;
  id_marker.pose.orientation.w = 1.;
  id_marker.pose.orientation.x = 0.;
  id_marker.pose.orientation.y = 0.;
  id_marker.pose.orientation.z = 0.;

  id_marker.lifetime = ros::Duration();
  id_marker.scale.z  = 1.0;
  id_marker.color.r  = 1.0;
  id_marker.color.g  = 1.0;
  id_marker.color.b  = 1.0;
  id_marker.color.a  = 1.0;

  marker_array.markers.push_back(id_marker);

  return marker_array;
}


//! Create a MarkerArray for a \a Area by invoking conversions to line strip
visualization_msgs::MarkerArray toMarkerArray(const lanelet::Area& area,
                                              const std::string& frame_id         = "/map",
                                              const std::string& namespace_prefix = "",
                                              double scale                        = 0.05,
                                              double color_r                      = 1.0,
                                              double color_g                      = 0.0,
                                              double color_b                      = 0.0,
                                              double color_a                      = 1.0,
                                              double lifetime                     = 0.0)
{
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker marker;

  double x_sum = 0.;
  double y_sum = 0.;
  int n        = 0;

  for (const auto& ob : area.outerBound())
  {
    ma.markers.push_back(toMarker(ob,
                                  frame_id,
                                  namespace_prefix,
                                  visualization_msgs::Marker::LINE_STRIP,
                                  scale,
                                  color_r,
                                  color_g,
                                  color_b,
                                  color_a,
                                  lifetime));

    for (const auto& p : ob)
    {
      x_sum += p.x();
      y_sum += p.y();
      ++n;
    }
  }

  std::stringstream ss;
  ss << area.id();
  // id
  visualization_msgs::Marker id_marker;
  id_marker.header.frame_id = frame_id;
  id_marker.header.stamp    = ros::Time();
  id_marker.ns              = "ids";
  id_marker.id              = area.id();
  id_marker.text            = ss.str();
  id_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
  id_marker.action          = visualization_msgs::Marker::ADD;
  id_marker.frame_locked    = true;

  n                            = std::max(n, 1);
  id_marker.pose.position.x    = x_sum / (double)n;
  id_marker.pose.position.y    = y_sum / (double)n;
  id_marker.pose.position.z    = 0.;
  id_marker.pose.orientation.w = 1.;
  id_marker.pose.orientation.x = 0.;
  id_marker.pose.orientation.y = 0.;
  id_marker.pose.orientation.z = 0.;

  id_marker.lifetime = ros::Duration();
  id_marker.scale.z  = 1.0;
  id_marker.color.r  = 1.0;
  id_marker.color.g  = 1.0;
  id_marker.color.b  = 1.0;
  id_marker.color.a  = 1.0;

  ma.markers.push_back(id_marker);


  return ma;
}


//! Create a MarkerArray for a \a ParkingSpace by invoking converions to line strip
visualization_msgs::MarkerArray toMarkerArray(const lanelet::Areas& areas,
                                              const std::string& frame_id         = "/map",
                                              const std::string& namespace_prefix = "",
                                              double scale                        = 0.05,
                                              double color_r                      = 1.0,
                                              double color_g                      = 0.0,
                                              double color_b                      = 0.0,
                                              double color_a                      = 1.0,
                                              double lifetime                     = 0.0)
{
  visualization_msgs::MarkerArray ma, ma_tmp;

  for (const auto& area : areas)
  {
    ma_tmp = toMarkerArray(
      area, frame_id, namespace_prefix, scale, color_r, color_g, color_b, color_a, lifetime);
    ma.markers.insert(ma.markers.end(), ma_tmp.markers.begin(), ma_tmp.markers.end());
  }

  return ma;
}



visualization_msgs::MarkerArray toMarkerArray(lanelet::LaneletMapPtr lanelet_map,
                                              const std::string& frame_id         = "/map",
                                              const std::string& namespace_prefix = "")
{
  visualization_msgs::MarkerArray ma, ma_tmp;

  // --------------- Lanelets -------------------
  for (auto& lanelet : lanelet_map->laneletLayer)
  {
    ma_tmp = toMarkerArray(
      lanelet, frame_id, namespace_prefix + "lanelets", 0.05, 1.0, 0.0, 0.0, 1.0, 0.0);
    ma.markers.insert(ma.markers.end(), ma_tmp.markers.begin(), ma_tmp.markers.end());
  }


  // --------------- Areas and Parking Spaces -------------------
  lanelet::Areas parking_spaces;
  for (const auto& area : lanelet_map->areaLayer)
  {
    if (area.hasAttribute("subtype") && area.attribute("subtype") == "parking")
    {
      ma_tmp =
        toMarkerArray(area, frame_id, namespace_prefix + "parking", 0.05, 0.0, 0.5, 1.0, 1.0, 0.0);
      ma.markers.insert(ma.markers.end(), ma_tmp.markers.begin(), ma_tmp.markers.end());
    }
    else
    {
      ma_tmp =
        toMarkerArray(area, frame_id, namespace_prefix, 0.1, 0.4, 0.8, 0.0, 1.0, 0.0);
      ma.markers.insert(ma.markers.end(), ma_tmp.markers.begin(), ma_tmp.markers.end());
    }

  }

  return ma;
}


} // namespace ros_lanelet2_visualization
#endif
