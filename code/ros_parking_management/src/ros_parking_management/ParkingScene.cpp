

#include <ros_parking_management/ParkingScene.h>
#include <ros_parking_management/ParkingSpace.h>
#include <ros_scene_prediction/RoutePrediction.h>
namespace ros_parking_management
{
ParkingScene::ParkingScene(std::string lanelet_filename, bool add_missing_reg_elem,
                           bool interpolate_centerline, bool interpolate_distances)
{
  m_scene.reset(new ros_scene_prediction::Scene(lanelet_filename, add_missing_reg_elem, interpolate_centerline,
                                interpolate_distances, 0.2,
                                lanelet::Origin({49.0142253, 8.4206021})));

  initializeParkingSpaces();
  std::cout << "Scence has initialized parking spaces " << m_parking_spaces.size() << std::endl;

  for (auto lanelet : m_scene->laneletMap()->laneletLayer)
  {
    auto accessible_relations = lanelet.regulatoryElementsAs<ros_parking_management::Accessible>();

    for (auto rel : accessible_relations)
    {
      lanelet::ConstLanelets llets = rel->getLanelets();
      lanelet::ConstAreas areas = rel->getAreas();
      std::cout << "lanelet " << lanelet.id() << " has accessible relation that has "
                << llets.size() << " lanelets and " << areas.size() << " areas" << std::endl;
    }
  }
}

void ParkingScene::initializeParkingSpaces()
{
  // get all parking spaces from the map
  lanelet::Areas parking_spaces;
  for (auto area : m_scene->laneletMap()->areaLayer)
  {
    if (area.hasAttribute("subtype") && area.attribute("subtype") == "parking")
    {
      auto reg_elements = area.regulatoryElementsAs<ros_parking_management::Accessible>();

      if (reg_elements.size() == 0)
      {
        std::cout << "Warning while initializing parking spaces: " << area.id()
                  << " is not accessible!" << std::endl;
      }
      else
      {
        parking_spaces.push_back(area);
      }
    }
  }

  bool charge_type_from_area = true;
  lanelet::Areas charge_type_areas;
  for (auto area : m_scene->laneletMap()->areaLayer)
  {
    if (area.hasAttribute("subtype") && area.attribute("subtype") == "charging_type_area")
    {
      charge_type_areas.push_back(area);
    }
  }

  // we assume that parking spaces consists of 4 points that describe a
  // rectangle
  for (auto ps : parking_spaces)
  {
    double length =
        (ps.outerBoundPolygon().basicPolygon()[0] - ps.outerBoundPolygon().basicPolygon()[1])
            .norm();
    double width =
        (ps.outerBoundPolygon().basicPolygon()[1] - ps.outerBoundPolygon().basicPolygon()[2])
            .norm();

    bool turn = false;
    if (width > length)
    {
      turn = true;
      double tmp = length;
      length = width;
      width = tmp;
    }

    geom::Pose<double, double> pose;
    pose.x =
        (ps.outerBoundPolygon().basicPolygon()[0] + ps.outerBoundPolygon().basicPolygon()[2]).x() /
        2.;
    pose.y =
        (ps.outerBoundPolygon().basicPolygon()[0] + ps.outerBoundPolygon().basicPolygon()[2]).y() /
        2.;
    pose.yaw = std::atan2(
        (ps.outerBoundPolygon().basicPolygon()[0] - ps.outerBoundPolygon().basicPolygon()[1]).y(),
        (ps.outerBoundPolygon().basicPolygon()[0] - ps.outerBoundPolygon().basicPolygon()[1]).x());

    if (turn)
    {
      pose.yaw += M_PI_2;
    }

    ParkingSpace::ParkingDirection dir;
    if (ps.hasAttribute("parking_direction"))
    {
      if (ps.attribute("parking_direction") == "both")
      {
        dir = ParkingSpace::ParkingDirection::BOTH;
      }
      else if (ps.attribute("parking_direction") == "forward")
      {
        dir = ParkingSpace::ParkingDirection::FORWARD;
      }
      else if (ps.attribute("parking_direction") == "backward")
      {
        dir = ParkingSpace::ParkingDirection::BACKWARD;
      }
    }
    else
    {
      dir = ParkingSpace::ParkingDirection::BOTH;
    }

    ChargeType type = ChargeType::NONE;
    if (charge_type_from_area)
    {
      for (auto type_area : charge_type_areas)
      {
        if (!boost::geometry::covered_by(pose.translation(), type_area.basicPolygonWithHoles2d()))
        {
          continue;
        }

        if (type_area.attribute("charge_type") == "electric")
        {
          type = ChargeType::ELECTRIC;
        }
        else if (type_area.attribute("charge_type") == "electric_fast")
        {
          type = ChargeType::ELECTRIC_FAST;
        }
        else if (type_area.attribute("charge_type") == "electric_inductive")
        {
          type = ChargeType::ELECTRIC_INDUCTIVE;
        }
      }
    }
    else
    {
      if (ps.hasAttribute("charge_type"))
      {
        if (ps.attribute("charge_type") == "electric")
        {
          type = ChargeType::ELECTRIC;
        }
        else if (ps.attribute("charge_type") == "electric_fast")
        {
          type = ChargeType::ELECTRIC_FAST;
        }
        else if (ps.attribute("charge_type") == "electric_inductive")
        {
          type = ChargeType::ELECTRIC_INDUCTIVE;
        }
      }
    }

    ParkingSpacePtr parking_space(new ParkingSpace(ps.id(), pose, length, width, type, dir, ps));

    m_parking_spaces.push_back(parking_space);
  }  // end for parking_spaces
}

ParkingSpaces ParkingScene::freeParkingSpaces()
{
  ParkingSpaces result;
  for (auto ps : m_parking_spaces)
  {
    if (ps->free())
    {
      result.push_back(ps);
    }
  }
  return result;
}

ParkingSpacePtr ParkingScene::closest(ParkingSpaces parking_spaces,
                                      geom::Pose<double, double> query_pose)
{
  double distance = std::numeric_limits<double>::max();

  ParkingSpacePtr result;
  for (auto ps : parking_spaces)
  {
    double d2 = (ps->pose().translation() - query_pose.translation()).squaredNorm();
    if (d2 < distance)
    {
      result = ps;
      distance = d2;
    }
  }

  return result;
}

lanelet::Ids ParkingScene::getRoute(ParkingSpacePtr &parkingspace_from,
                                    const lanelet::ConstLanelet &lanelet_to)
{
  lanelet::Ids result;

  if (parkingspace_from->area().regulatoryElementsAs<ros_parking_management::Accessible>().size() ==
      0)
  {
    std::cout << "ERROR: Parking space " << parkingspace_from->id() << " is not accessible!"
              << std::endl;
    throw std::runtime_error("Parking space is not accessible. Unable to calculate route.");;
  }

  lanelet::Ids lanelets_from_ids;
  for (auto &llet : parkingspace_from->area()
                        .regulatoryElementsAs<ros_parking_management::Accessible>()
                        .front()
                        ->getLanelets())
  {
    lanelets_from_ids.push_back(llet.id());
  }

  return ros_scene_prediction::shortestRoute(m_scene, lanelets_from_ids, {lanelet_to.id()});
}

lanelet::Ids ParkingScene::getRoute(ParkingSpacePtr &parkingspace_from,
                                    const ParkingSpacePtr &parkingspace_to)
{
  lanelet::Ids result;

  if (parkingspace_from->area().regulatoryElementsAs<ros_parking_management::Accessible>().size() ==
      0)
  {
    std::cout << "ERROR: Parking space " << parkingspace_from->id() << " is not accessible!"
              << std::endl;
    throw std::runtime_error("Parking space is not accessible. Unable to calculate route.");;
  }

  if (parkingspace_to->area().regulatoryElementsAs<ros_parking_management::Accessible>().size() ==
      0)
  {
    std::cout << "ERROR: Parking space " << parkingspace_to->id() << " is not accessible!"
              << std::endl;
    throw std::runtime_error("Parking space is not accessible. Unable to calculate route.");;
  }

  lanelet::Ids lanelets_from_ids;
  for (auto &llet : parkingspace_from->area()
                        .regulatoryElementsAs<ros_parking_management::Accessible>()
                        .front()
                        ->getLanelets())
  {
    lanelets_from_ids.push_back(llet.id());
  }

  lanelet::Ids lanelets_to_ids;
  for (auto &llet : parkingspace_to->area()
                        .regulatoryElementsAs<ros_parking_management::Accessible>()
                        .front()
                        ->getLanelets())
  {
    lanelets_to_ids.push_back(llet.id());
  }

  return ros_scene_prediction::shortestRoute(m_scene, lanelets_from_ids, lanelets_to_ids);
}

lanelet::Ids ParkingScene::getRoute(const lanelet::ConstLanelet &lanelet_from,
                                    ParkingSpacePtr &parkingspace_to)
{
  lanelet::Ids result;

  if (parkingspace_to->area().regulatoryElementsAs<ros_parking_management::Accessible>().size() ==
      0)
  {
    std::cout << "ERROR: Parking space " << parkingspace_to->id() << " is not accessible!"
              << std::endl;
    throw std::runtime_error("Parking space is not accessible. Unable to calculate route.");;
  }

  lanelet::Ids lanelets_to_ids;
  for (auto &llet : parkingspace_to->area()
                        .regulatoryElementsAs<ros_parking_management::Accessible>()
                        .front()
                        ->getLanelets())
  {
    lanelets_to_ids.push_back(llet.id());
  }

  return ros_scene_prediction::shortestRoute(m_scene, {lanelet_from.id()}, lanelets_to_ids);
}

lanelet::Ids ParkingScene::getRoute(geom::Pose<double, double> &pose_from,
                                    const ParkingSpacePtr &parkingspace_to)
{
  auto lanelet_candidates_from = ros_scene_prediction::LaneletMapper::determineLaneletMapping(
      m_scene, pose_from.translation(), pose_from.yaw);

  std::vector<ros_scene_prediction::Id> lanelets_from_ids;
  for (auto id : lanelet_candidates_from.m_is_following_lanelet_ids)
  {
    lanelets_from_ids.push_back(id.second);
  }

  if (lanelets_from_ids.empty())
  {
    std::cout << "No matching lanelet for given pose!" << std::endl;
    return lanelet::Ids();
  }

  lanelet::Ids lanelets_to_ids;
  for (auto &llet : parkingspace_to->area()
                        .regulatoryElementsAs<ros_parking_management::Accessible>()
                        .front()
                        ->getLanelets())
  {
    lanelets_to_ids.push_back(llet.id());
  }

  return ros_scene_prediction::shortestRoute(m_scene, lanelets_from_ids, lanelets_to_ids);
}

void plotScene(ParkingScene &scene, const std::string filenameprefix)
{
  ros_scene_prediction::plotScene(*(scene.scene()), filenameprefix);

  std::stringstream ss_parking_spaces;
  for (auto ps : scene.parkingspaces())
  {
    for (auto p : ps->area().outerBoundPolygon())
    {
      ss_parking_spaces << p.x() << " " << p.y() << std::endl;
    }
    ss_parking_spaces << std::endl;
  }

  writeStringStreamToFile(filenameprefix + "_parkingspaces.gpldata", ss_parking_spaces);
}

}  // namespace ros_parking_management
