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
 * \date    2020-01-22
 *
 *
 */
//----------------------------------------------------------------------
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <ros/ros.h>

#include <gps_common/GPSFix.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>

//#include <lanelet2_routing/Route.h>
//#include <lanelet2_routing/RoutingCost.h>
//#include <lanelet2_routing/RoutingGraph.h>
//#include <lanelet2_routing/RoutingGraphContainer.h>
//#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <ros_lanelet2_visualization/Lanelet2Visualization.h>


void writeStringToFile(const std::string filename, const std::string& text, bool verbose = true)
{
  std::ofstream output_file;
  output_file.open(filename, std::ios::out);
  if (output_file.is_open())
  {
    output_file << text << std::endl;
    output_file.close();
  }
  else
  {
    std::cout << "Error while trying to write to file: " << filename
              << " file is not open. Check if directory exists or file is not writeable."
              << std::endl;
  }
}


void writeStringStreamToFile(const std::string filename,
                             std::stringstream& stream,
                             bool verbose = true)
{
  writeStringToFile(filename, stream.str().c_str(), verbose);
}

int main(int argc, char* argv[])
{

  std::string lanelet_filename = "/path/to/lanelet/technologiepark_lanelet2.osm";
  double gnss_reference_lat = 49.;
  double gnss_reference_lon = 8.4;


  lanelet::projection::UtmProjector projector(
    lanelet::Origin({gnss_reference_lat, gnss_reference_lon}));
  lanelet::LaneletMapPtr map = load(lanelet_filename, projector);

  std::cout << "Points: " << map->pointLayer.size() << std::endl;
  std::cout << "Lanelets: " << map->laneletLayer.size() << std::endl;
  std::cout << "Lanelets: " << map->lineStringLayer.size() << std::endl;
  std::cout << "Areas: " << map->areaLayer.size() << std::endl;

  std::stringstream ss;
  for (auto& ls : map->lineStringLayer)
  {
    for (auto& p : ls)
    {
      ss << p.x() << " " << p.y() << std::endl;
    }
    ss << std::endl;
  }


  writeStringStreamToFile("/tmp/laneletmap.gpldata", ss);


  ros::init(argc, argv, "Lanelet2MapVisualizer");

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");


  std::string frame_id = "/map";


  // --- Subscribers ---
  // ros::Subscriber gnss_reference_sub =
  //   nh.subscribe<gps_common::GPSFix>("/gnss_reference", 0, gnssReferenceCb);

  int loop_rate = 10;
  priv_nh.param<int>("loop_rate", loop_rate, loop_rate);
  ros::Rate rate(loop_rate);

  // -------------- Linestring --------------------------------

  ros::Publisher linestring_marker_pub =
    nh.advertise<visualization_msgs::Marker>("linestring_marker", 0, true);

  visualization_msgs::Marker linestring_marker =
    ros_lanelet2_visualization::toMarker(map->laneletLayer.begin()->leftBound(),
                                         "/map",
                                         "lanelet line strip",
                                         visualization_msgs::Marker::LINE_STRIP,
                                         1.05,
                                         1.0,
                                         1.0,
                                         0.0,
                                         1.0,
                                         0.0);


  // -------------- lanelet --------------------------------
  ros::Publisher lanelet_marray_pub =
    nh.advertise<visualization_msgs::MarkerArray>("lanelet_marker_array", 0, true);

  visualization_msgs::MarkerArray lanelet_marker_array = ros_lanelet2_visualization::toMarkerArray(
    *(map->laneletLayer.begin()), "/map", "lanelet", 0.5, 0.0, 1.0, 1.0, 1.0, 0.0);

  // -------------- lanelet map --------------------------------

  ros::Publisher laneletmap_marray_pub =
    nh.advertise<visualization_msgs::MarkerArray>("lanelet_map_marker_array", 0, true);

  visualization_msgs::MarkerArray laneletmap_marker_array =
    ros_lanelet2_visualization::toMarkerArray(map, "/map", "laneletmap");

  // -------------- Areas --------------------------------

  lanelet::Areas parking_spaces;
  for (auto& area : map->areaLayer)
  {
    if (area.hasAttribute("subtype") && area.attributes()["subtype"] == "parking")
    {
      parking_spaces.push_back(area);
    }
  }

  std::cout << "Parking spaces " << parking_spaces.size() << std::endl;

  ros::Publisher parking_space_marray_pub =
    nh.advertise<visualization_msgs::MarkerArray>("parking_spaces_marker_array", 0, true);

  visualization_msgs::MarkerArray parking_space_marker_array =
    ros_lanelet2_visualization::toMarkerArray(
      parking_spaces, "/map", "parking", 0.05, 0.0, 0.5, 1.0, 1.0, 0.0);

  // ---------------------------------------------------

  ROS_WARN_STREAM(ros::this_node::getName() << ": Ready.");
  while (ros::ok())
  {
    // receive subscriber callbacks
    ros::spinOnce();

    lanelet_marray_pub.publish(lanelet_marker_array);
    linestring_marker_pub.publish(linestring_marker);
    laneletmap_marray_pub.publish(laneletmap_marker_array);
    parking_space_marray_pub.publish(parking_space_marker_array);
    // sleep the remaining time
    rate.sleep();
  }

  ROS_WARN_STREAM(ros::this_node::getName() << ": Terminating.");
  return 0;
}
