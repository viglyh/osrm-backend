#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/turn_lane_matcher.hpp"
#include "util/typedefs.hpp"

#include <cstdint>
#include <iomanip>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/numeric/conversion/cast.hpp>

namespace osrm
{
namespace extractor
{
namespace guidance
{

namespace detail
{

DirectionModifier::Enum getMatchingModifier(const std::string &tag)
{
    const constexpr char *tag_by_modifier[] = {"reverse",
                                               "sharp_right",
                                               "right",
                                               "slight_right",
                                               "through",
                                               "slight_left",
                                               "left",
                                               "sharp_left",
                                               "merge_left",
                                               "merge_right"};
    const auto index =
        std::distance(tag_by_modifier, std::find(tag_by_modifier, tag_by_modifier + 10, tag));

    const constexpr DirectionModifier::Enum modifiers[] = {DirectionModifier::UTurn,
                                                           DirectionModifier::SharpRight,
                                                           DirectionModifier::Right,
                                                           DirectionModifier::SlightRight,
                                                           DirectionModifier::Straight,
                                                           DirectionModifier::SlightLeft,
                                                           DirectionModifier::Left,
                                                           DirectionModifier::SharpLeft,
                                                           DirectionModifier::SlightLeft,
                                                           DirectionModifier::SlightRight};

    std::cout << "Tag: " << tag << std::endl;
    BOOST_ASSERT(index < 10);
    return modifiers[index];
}

// find turn that is closest to a given angle
std::size_t findClosestTurnIndex(const Intersection &intersection, const double angle)
{
    const auto itr = std::min_element(intersection.begin(),
                                      intersection.end(),
                                      [angle](const ConnectedRoad &lhs, const ConnectedRoad &rhs) {
                                          return angularDeviation(lhs.turn.angle, angle) <
                                                 angularDeviation(rhs.turn.angle, angle);
                                      });
    return std::distance(intersection.begin(), itr);
}

typename Intersection::const_iterator findBestMatch(const std::string &tag,
                                                    const Intersection &intersection)
{
    const constexpr double idealized_turn_angles[] = {0, 35, 90, 135, 180, 225, 270, 315};
    const auto idealized_angle = idealized_turn_angles[getMatchingModifier(tag)];
    return std::min_element(intersection.begin(),
                            intersection.end(),
                            [idealized_angle](const ConnectedRoad &lhs, const ConnectedRoad &rhs) {
                                // if the entry allowed flags don't match, we select the one with
                                // entry allowed set to true
                                if (lhs.entry_allowed != rhs.entry_allowed)
                                    return lhs.entry_allowed;

                                return angularDeviation(idealized_angle, lhs.turn.angle) <
                                       angularDeviation(idealized_angle, rhs.turn.angle);
                            });
}

bool isValidMatch(const std::string &tag, const TurnInstruction &instruction)
{
    if (tag == "reverse")
    {
        return instruction.direction_modifier == DirectionModifier::UTurn;
    }
    else if (tag == "sharp_right" || tag == "right" || tag == "slight_right")
    {
        return instruction.direction_modifier == DirectionModifier::SlightRight ||
               instruction.direction_modifier == DirectionModifier::Right ||
               instruction.direction_modifier == DirectionModifier::SharpRight;
    }
    else if (tag == "through")
    {
        return instruction.direction_modifier == DirectionModifier::Straight ||
               instruction.type == TurnType::Suppressed || instruction.type == TurnType::NewName;
    }
    else if (tag == "slight_left" || tag == "left" || tag == "sharp_left")
    {
        return instruction.direction_modifier == DirectionModifier::SlightLeft ||
               instruction.direction_modifier == DirectionModifier::Left ||
               instruction.direction_modifier == DirectionModifier::SharpLeft;
    }
    return false;
}

std::size_t getNumberOfTurns(const Intersection &intersection)
{
    return std::count_if(intersection.begin(), intersection.end(), [](const ConnectedRoad &road) {
        return road.entry_allowed;
    });
}

} // namespace detail

bool TurnLaneMatcher::TurnLaneData::operator<(const TurnLaneMatcher::TurnLaneData &other) const
{
    if (from < other.from)
        return true;
    if (from > other.from)
        return false;

    if (to < other.to)
        return true;
    if (to > other.to)
        return false;

    const constexpr char *tag_by_modifier[] = {"sharp_right",
                                               "right",
                                               "slight_right",
                                               "through",
                                               "slight_left",
                                               "left",
                                               "sharp_left",
                                               "reverse"};
    return std::find(tag_by_modifier, tag_by_modifier + 8, this->tag) <
           std::find(tag_by_modifier, tag_by_modifier + 8, other.tag);
}

std::size_t findTag(const std::string &tag, const TurnLaneMatcher::LaneDataVector &data)
{
    return std::distance(
        data.begin(),
        std::find_if(data.begin(), data.end(), [&](const TurnLaneMatcher::TurnLaneData &lane_data) {
            return tag == lane_data.tag;
        }));
}

TurnLaneMatcher::TurnLaneMatcher(const util::NodeBasedDynamicGraph &node_based_graph,
                                 const util::NameTable &turn_lane_strings,
                                 const std::vector<QueryNode> &node_info_list,
                                 const TurnAnalysis &turn_analysis)
    : node_based_graph(node_based_graph), turn_lane_strings(turn_lane_strings),
      node_info_list(node_info_list), turn_analysis(turn_analysis)
{
}

/*
    Turn lanes are given in the form of strings that closely correspond to the direction modifiers
   we use for our turn types. However, we still cannot simply perform a 1:1 assignment.

    In this function we match the turn lane strings to the actual turns.
    The input contains of a string of the format |left|through;right|right| for a setup like

    ----------
    A -^
    ----------
    B -> -v
    ----------
    C -v
    ----------

    For this setup, we coul get a set of turns of the form

    (130, turn slight right), (180, ramp straight), (320, turn sharp left)

    Here we need to augment to:
    (130, turn slight right, <3,2,0>) (180,ramp straight,<3,1,1>), and (320, turn sharp left,
   <3,1,2>)
 */
Intersection TurnLaneMatcher::assignTurnLanes(const EdgeID via_edge,
                                              Intersection intersection) const
{
    const auto &data = node_based_graph.GetEdgeData(via_edge);
    const auto turn_lane_string =
        data.lane_id != INVALID_LANEID ? turn_lane_strings.GetNameForID(data.lane_id) : "";
    std::cout << "Edge: " << via_edge << " String: \"" << turn_lane_string << "\"" << std::endl;

    // FIXME this is a workaround due to https://github.com/cucumber/cucumber-js/issues/417,
    // need to switch statements when fixed
    // const auto num_lanes = std::count(turn_lane_string.begin(), turn_lane_string.end(), '|') + 1;
    auto countLanes = [](const std::string &turn_lane_string) {
        return boost::numeric_cast<LaneID>(
            std::count(turn_lane_string.begin(), turn_lane_string.end(), '|') + 1 +
            std::count(turn_lane_string.begin(), turn_lane_string.end(), '&'));
    };

    auto getNextTag = [](std::string &string, const char *separators) {
        auto pos = string.find_last_of(separators);
        auto result = pos != std::string::npos ? string.substr(pos + 1) : string;

        string.resize(pos == std::string::npos ? 0 : pos);
        return result;
    };

    auto setLaneData = [&](LaneMap &map, std::string lane, const LaneID current_lane) {
        do
        {
            auto identifier = getNextTag(lane, ";");
            if (identifier.empty())
                identifier = "none";
            auto map_iterator = map.find(identifier);
            if (map_iterator == map.end())
                map[identifier] = std::make_pair(current_lane, current_lane);
            else
            {
                map_iterator->second.second = current_lane;
            }
        } while (!lane.empty());
    };

    auto convertLaneStringToData = [&](const LaneID num_lanes, std::string lane_string) {
        LaneMap lane_map;
        LaneID lane_nr = 0;
        do
        {
            // FIXME this is a cucumber workaround, since escaping does not work properly in
            // cucumber.js (see https://github.com/cucumber/cucumber-js/issues/417). Needs to be
            // changed to "|" only, when the bug is fixed
            auto lane = getNextTag(lane_string, "|&");
            setLaneData(lane_map, lane, lane_nr);
            ++lane_nr;
        } while (lane_nr < num_lanes);

        LaneDataVector lane_data;
        for (const auto tag : lane_map)
        {
            lane_data.push_back({tag.first, tag.second.first, tag.second.second});
        }

        std::sort(lane_data.begin(), lane_data.end());

        return lane_data;
    };

    Intersection previous_intersection;
    EdgeID previous_id = SPECIAL_EDGEID;
    const auto previous_lane_string = [&]() -> std::string {
        /* We need to find the intersection that is located prior to via_edge.
         *
         * NODE_X  -> PREVIOUS_ID            -> NODE -> VIA_EDGE -> INTERSECTION
         * NODE_X? <- STRAIGHTMOST           <- NODE <- UTURN
         * NODE_X? -> UTURN == PREVIOUSE_ID? -> NODE -> VIA_EDGE
         *
         * To do so, we first get the intersection at NODE and find the straightmost turn from that
         * node. This will result in NODE_X. The uturn in the intersection at NODE_X should be
         * PREVIOUS_ID. To verify that find, we check the intersection using our PREVIOUS_ID
         * candidate to check the intersection at NODE for via_edge
         */
        std::cout << "Distance: " << data.distance << std::endl;
        const constexpr double COMBINE_DISTANCE_CUTOFF = 30;
        if (data.distance > COMBINE_DISTANCE_CUTOFF)
            return {};

        const auto uturn_id = intersection[0].turn.eid;
        const auto source_intersection =
            turn_analysis.getIntersection(node_based_graph.GetTarget(via_edge), uturn_id);
        // check for a straight turn

        const auto straightmost_index =
            detail::findClosestTurnIndex(source_intersection, STRAIGHT_ANGLE);
        if (angularDeviation(source_intersection[straightmost_index].turn.angle, STRAIGHT_ANGLE) >
            FUZZY_ANGLE_DIFFERENCE)
            return {};

        const auto node_x =
            node_based_graph.GetTarget(source_intersection[straightmost_index].turn.eid);
        const auto intersection_at_straight = turn_analysis.getIntersection(
            node_based_graph.GetTarget(uturn_id), source_intersection[straightmost_index].turn.eid);

        // now check that the u-turn at the given intersection connects to via-edge
        previous_id = intersection_at_straight[0].turn.eid;
        previous_intersection = turn_analysis.getIntersection(node_x, previous_id);

        const auto check_via_edge =
            previous_intersection[detail::findClosestTurnIndex(previous_intersection,
                                                               STRAIGHT_ANGLE)]
                .turn.eid;
        if (check_via_edge != via_edge)
            return {};

        const auto &previous_data = node_based_graph.GetEdgeData(previous_id);
        auto previous_string = previous_data.lane_id != INVALID_LANEID
                                   ? turn_lane_strings.GetNameForID(previous_data.lane_id)
                                   : "";

        if (previous_string.empty())
            return previous_string;

        previous_intersection =
            turn_analysis.assignTurnTypes(node_x, previous_id, std::move(previous_intersection));

        auto previous_lane_data =
            convertLaneStringToData(countLanes(previous_string), previous_string);
        if (isSimpleIntersection(previous_lane_data, previous_intersection))
            return "";

        std::cout << "Previous: " << previous_string << std::endl;
        for (auto road : previous_intersection)
            std::cout << "\t" << toString(road) << std::endl;

        return previous_string;
    }();

    std::cout << "Previous String: \"" << previous_lane_string << "\"" << std::endl;

    if (turn_lane_string.empty() && previous_lane_string.empty())
    {
        for (auto &road : intersection)
            road.turn.instruction.lane_tupel = {0, INVALID_LANEID};

        return intersection;
    }
    LaneID num_lanes = countLanes(turn_lane_string);

    auto lane_data = convertLaneStringToData(num_lanes, turn_lane_string);

    // might be reasonable to handle multiple turns, if we know of a sequence of lanes
    // e.g. one direction per lane, if three lanes and right, through, left available
    if (!turn_lane_string.empty() && lane_data.size() == 1 && lane_data[0].tag == "none")
    {
        for (auto &road : intersection)
            road.turn.instruction.lane_tupel = {0, INVALID_LANEID};
        return intersection;
    }

    // check whether we are at a simple intersection
    bool is_simple = isSimpleIntersection(lane_data, intersection);

    std::cout << "Lane String: " << turn_lane_string
              << " Simple: " << (is_simple ? "true" : "false") << std::endl;

    auto node = node_based_graph.GetTarget(via_edge);
    auto coordinate = node_info_list[node];

    if (is_simple && !turn_lane_string.empty())
    {
        lane_data = handleNoneValueAtSimpleTurn(
            node_based_graph.GetTarget(via_edge), std::move(lane_data), intersection);
        return simpleMatchTuplesToTurns(std::move(intersection), num_lanes, lane_data);
    }
    else if (turn_lane_string.empty() && !previous_lane_string.empty())
    {
        std::cout << "Using previous data\n" << std::endl;
        num_lanes = countLanes(previous_lane_string);
        lane_data = convertLaneStringToData(num_lanes, previous_lane_string);
        is_simple = isSimpleIntersection(lane_data, intersection);

        std::cout << "Lane String: " << turn_lane_string
                  << " Simple: " << (is_simple ? "true" : "false") << std::endl;
        if (is_simple && !previous_lane_string.empty())
        {
            lane_data = handleNoneValueAtSimpleTurn(
                node_based_graph.GetTarget(via_edge), std::move(lane_data), intersection);
            return simpleMatchTuplesToTurns(std::move(intersection), num_lanes, lane_data);
        }
        else if (!previous_lane_string.empty())
        {
            std::cout << "Handling non-simple instructions" << std::endl;
            if (lane_data.size() > detail::getNumberOfTurns(intersection))
            {
                lane_data = partitionLaneData(node_based_graph.GetTarget(previous_id),
                                              std::move(lane_data),
                                              previous_intersection)
                                .second;

                for (auto data : lane_data)
                    std::cout << "Tag: " << data.tag << " " << data.from << " " << data.to
                              << std::endl;

                // check if we were successfull in trimming
                if (lane_data.size() == detail::getNumberOfTurns(intersection))
                {
                    lane_data = handleNoneValueAtSimpleTurn(
                        node_based_graph.GetTarget(via_edge), std::move(lane_data), intersection);

                    return simpleMatchTuplesToTurns(std::move(intersection), num_lanes, lane_data);
                }
            }
        }
    }
    else if (!turn_lane_string.empty())
    {
        /*
            We need to check whether the turn lanes have to be propagated further at some
           points.
            Some turn lanes are given on a segment prior to the one where the turn actually
           happens.
            These have to be pushed along to the segment where the data is actually used.

                      |    |           |    |
            ----------      -----------      -----



            ----------      -----------      -----
                      |    |           |    |
                      | vv |           | ^^ |
            ----------      -----------      ------
             (a)-----^
             (b)----->
             (c)-----v
            ----------      -----------      ------
                     |      |          |     |

            Both (a) and (b) are targeting not only the intersection they are at. The correct
           representation for routing is:

                      |    |           |    |
            ----------      -----------      -----



            ----------      -----------      -----
                      |    |           |    |
                      | vv |           | ^^ |
            ----------      -----------      ------
             (a)-------------------------^
             (b)----->      ---------->
             (c)-----v
            ----------      -----------      ------
                     |      |          |     |


        */

        std::cout << "Handling non-simple instructions" << std::endl;
        if (lane_data.size() > detail::getNumberOfTurns(intersection))
        {
            lane_data = partitionLaneData(node_based_graph.GetTarget(via_edge),
                                          std::move(lane_data),
                                          intersection)
                            .first;

            // check if we were successfull in trimming
            if (lane_data.size() == detail::getNumberOfTurns(intersection))
            {
                lane_data = handleNoneValueAtSimpleTurn(
                    node_based_graph.GetTarget(via_edge), std::move(lane_data), intersection);

                return simpleMatchTuplesToTurns(std::move(intersection), num_lanes, lane_data);
            }
        }

        /*
        std::cout << "Skipping complex intersection, for now (" << std::setprecision(12)
                  << util::toFloating(coordinate.lat) << " " << util::toFloating(coordinate.lon)
                  << ") " << turn_lane_string << std::endl;
        */
        for (auto &road : intersection)
            road.turn.instruction.lane_tupel = {0, INVALID_LANEID};

        return intersection;
    }

    // Now assign the turn lanes to their respective turns

    /*
    std::cout << "Location: " << std::setprecision(12) << util::toFloating(coordinate.lat) << "
    "
              << util::toFloating(coordinate.lon) << std::endl;
    for (auto tag : lane_data)
        std::cout << "Lane Information: " << tag.tag << " " << tag.from << "-" << tag.to
                  << std::endl;

    std::cout << "Lane Data: " << turn_lane_strings.GetNameForID(data.lane_id) << std::endl;
    for (const auto &turn : intersection)
    {
        std::cout << "Turn: " << toString(turn) << std::endl;
    }
    */
}

/*
    Lanes can have the tag none. While a nice feature for visibility, it is a terrible feature
   for
   parsing. None can be part of neighboring turns, or not. We have to look at both the
   intersection
   and the lane data to see what turns we have to augment by the none-lanes
 */
TurnLaneMatcher::LaneDataVector TurnLaneMatcher::handleNoneValueAtSimpleTurn(
    const NodeID at, LaneDataVector lane_data, const Intersection &intersection) const
{
    if (intersection.empty() || lane_data.empty())
        return lane_data;

    // FIXME all this needs to consider the number of lanes at the target to ensure that we
    // augment
    // lanes correctly, if the target lane allows for more turns
    //
    // -----------------
    //
    // -----        ----
    //  -v          |
    // -----        |
    //      |   |   |
    //
    // A situation like this would allow a right turn from the through lane.
    //
    // -----------------
    //
    // -----    --------
    //  -v      |
    // -----    |
    //      |   |
    //
    // Here, the number of lanes in the right road would not allow turns from both lanes, but
    // only
    // from the right one

    bool has_right = false;
    bool has_through = false;
    bool has_left = false;
    std::size_t connection_count = 0;
    for (const auto &road : intersection)
    {
        std::cout << "Road: " << toString(road) << std::endl;
        if (!road.entry_allowed)
            continue;

        ++connection_count;
        const auto modifier = road.turn.instruction.direction_modifier;
        has_right |= modifier == DirectionModifier::Right;
        has_right |= modifier == DirectionModifier::SlightRight;
        has_right |= modifier == DirectionModifier::SharpRight;
        has_through |= modifier == DirectionModifier::Straight;
        has_left |= modifier == DirectionModifier::Left;
        has_left |= modifier == DirectionModifier::SlightLeft;
        has_left |= modifier == DirectionModifier::SharpLeft;
    }

    if (intersection[0].entry_allowed)
        --connection_count;

    const constexpr char *tag_by_modifier[] = {"reverse",
                                               "sharp_right",
                                               "right",
                                               "slight_right",
                                               "through",
                                               "slight_left",
                                               "left",
                                               "sharp_left"};

    // TODO check for impossible turns to see whether the turn lane is at the correct place

    for (std::size_t index = 0; index < lane_data.size(); ++index)
    {
        if (lane_data[index].tag == "none")
        {
            bool print = true;
            // we have to create multiple turns
            if (connection_count > lane_data.size())
            {
                // a none-turn is allowing multiple turns. we have to add a lane-data entry for
                // every possible turn. This should, hopefully, only be the case for single lane
                // entries?

                // looking at the left side first
                const auto range = [&]() {
                    if (index == 0)
                    {
                        // find first connection_count - lane_data.size() valid turns
                        std::size_t count = 0;
                        for (std::size_t intersection_index = 1;
                             intersection_index < intersection.size();
                             ++intersection_index)
                        {
                            count +=
                                static_cast<int>(intersection[intersection_index].entry_allowed);
                            if (count > connection_count - lane_data.size())
                                return std::make_pair(std::size_t{1}, intersection_index + 1);
                        }
                    }
                    else if (index + 1 == lane_data.size())
                    {
                        BOOST_ASSERT(!lane_data.empty());
                        // find last connection-count - last_data.size() valid turns
                        std::size_t count = 0;
                        for (std::size_t intersection_index = intersection.size() - 1;
                             intersection_index > 0;
                             --intersection_index)
                        {
                            count +=
                                static_cast<int>(intersection[intersection_index].entry_allowed);
                            if (count > connection_count - lane_data.size())
                                return std::make_pair(intersection_index, intersection.size());
                        }
                    }
                    else
                    {
                        // skip the first #index valid turns, find next connection_count -
                        // lane_data.size() valid ones

                        std::size_t begin = 1, count = 0, intersection_index;
                        for (intersection_index = 1; intersection_index < intersection.size();
                             ++intersection_index)
                        {
                            count +=
                                static_cast<int>(intersection[intersection_index].entry_allowed);
                            // if we reach the amount of
                            if (count >= index)
                            {
                                begin = intersection_index + 1;
                                break;
                            }
                        }

                        // reset count to find the number of necessary entries
                        count = 0;
                        for (intersection_index = begin; intersection_index < intersection.size();
                             ++intersection_index)
                        {
                            count +=
                                static_cast<int>(intersection[intersection_index].entry_allowed);
                            if (count > connection_count - lane_data.size())
                            {
                                return std::make_pair(begin, intersection_index + 1);
                            }
                        }
                    }
                    // this should, theoretically, never be reached
                    BOOST_ASSERT(false);
                    return std::make_pair(std::size_t{0}, std::size_t{0});
                }();
                std::cout << "Missing " << connection_count - lane_data.size() << " lane items."
                          << std::endl;
                for (auto intersection_index = range.first; intersection_index < range.second;
                     ++intersection_index)
                {
                    if (intersection[intersection_index].entry_allowed)
                    {
                        // FIXME this probably can be only a subset of these turns here?
                        lane_data.push_back(
                            {tag_by_modifier[intersection[intersection_index]
                                                 .turn.instruction.direction_modifier],
                             lane_data[index].from,
                             lane_data[index].to});
                        std::cout << "Added: " << lane_data.back().tag << " "
                                  << (int)lane_data.back().from << " " << (int)lane_data.back().to
                                  << std::endl;
                    }
                }
                lane_data.erase(lane_data.begin() + index);
                std::sort(lane_data.begin(), lane_data.end());
            }
            // we have to reduce it, assigning it to neighboring turns
            else if (connection_count < lane_data.size())
            {
                std::cout << "Connections: " << connection_count << std::endl;
                std::cout << "Input" << std::endl;
                for (auto tag : lane_data)
                    std::cout << "Lane Information: " << tag.tag << " " << (int)tag.from << "-"
                              << (int)tag.to << std::endl;

                // a prerequisite is simple turns. Larger differences should not end up here
                BOOST_ASSERT(connection_count + 1 == lane_data.size());
                // an additional line at the side is only reasonable if it is targeting public
                // service vehicles. Otherwise, we should not have it
                // TODO what about lane numbering. Should we count differently?
                if (index == 0 || index + 1 == lane_data.size())
                {
                    // FIXME not augment, if this is a psv lane only
                    if (index == 0)
                    {
                        lane_data[1].from = lane_data[0].from;
                    }
                    else
                    {
                        lane_data[index - 1].to = lane_data[index].to;
                    }
                    lane_data.erase(lane_data.begin() + index);
                }
                else if (lane_data[index].to - lane_data[index].from <= 1)
                {
                    lane_data[index - 1].to = lane_data[index].from;
                    lane_data[index + 1].from = lane_data[index].to;

                    lane_data.erase(lane_data.begin() + index);
                }
            }
            // we have to rename and possibly augment existing ones. The pure count remains the
            // same.
            else
            {
                // find missing tag and augment neighboring, if possible
                if (index == 0)
                {
                    if (has_right &&
                        (lane_data.size() == 1 || (lane_data[index + 1].tag != "sharp_right" &&
                                                   lane_data[index + 1].tag != "right")))
                    {
                        lane_data[index].tag = "right";
                        if (lane_data.size() > 1 && lane_data[index + 1].tag == "through")
                        {
                            lane_data[index + 1].from = lane_data[index].from;
                            // turning right through a possible through lane is not possible
                            lane_data[index].to = lane_data[index].from;
                        }
                    }
                    else if (has_through &&
                             (lane_data.size() == 1 || lane_data[index + 1].tag != "through"))
                    {
                        lane_data[index].tag = "through";
                    }
                }
                else if (index + 1 == lane_data.size())
                {
                    if (has_left && ((lane_data[index - 1].tag != "sharp_left" &&
                                      lane_data[index - 1].tag != "left")))
                    {
                        lane_data[index].tag = "left";
                        if (lane_data[index - 1].tag == "through")
                        {
                            lane_data[index - 1].to = lane_data[index].to;
                            // turning left through a possible through lane is not possible
                            lane_data[index].from = lane_data[index].to;
                        }
                    }
                    else if (has_through && lane_data[index - 1].tag != "through")
                    {
                        lane_data[index].tag = "through";
                    }
                }
                else
                {
                    if ((lane_data[index + 1].tag == "left" ||
                         lane_data[index + 1].tag == "slight_left" ||
                         lane_data[index + 1].tag == "sharp_left") &&
                        (lane_data[index - 1].tag == "right" ||
                         lane_data[index - 1].tag == "slight_right" ||
                         lane_data[index - 1].tag == "sharp_right"))
                    {
                        lane_data[index].tag = "through";
                    }
                }
            }
            std::sort(lane_data.begin(), lane_data.end());

            if (print)
            {
                auto coordinate = node_info_list[at];

                std::cout << "Location: " << std::setprecision(12)
                          << util::toFloating(coordinate.lat) << " "
                          << util::toFloating(coordinate.lon) << std::endl;

                std::cout << "Output" << std::endl;
                for (auto tag : lane_data)
                    std::cout << "Lane Information: " << tag.tag << " " << (int)tag.from << "-"
                              << (int)tag.to << std::endl;

                for (const auto &turn : intersection)
                {
                    std::cout << "Turn: " << toString(turn) << std::endl;
                }
            }
            break;
        }
    }

    // BOOST_ASSERT( lane_data.size() + 1 >= intersection.size() );
    return lane_data;
}

/* A simple intersection does not depend on the next intersection coming up. This is important
 * for
 * turn lanes, since traffic signals and/or segregated a intersection can influence the
 * interpretation of turn-lanes at a given turn.
 *
 * Here we check for a simple intersection. A simple intersection has a long enough segment
 * following
 * the turn, offers no straight turn, or only non-trivial turn operations.
 */
bool TurnLaneMatcher::isSimpleIntersection(const LaneDataVector &lane_data,
                                           const Intersection &intersection) const
{
    // if we are on a straight road, turn lanes are only reasonable in connection to the next
    // intersection, or in case of a merge. If not all but one (straight) are merges, we don't
    // consider the intersection simple
    if (intersection.size() == 2)
    {
        std::cout << "Checking for merge" << std::endl;
        return std::count_if(
                   lane_data.begin(),
                   lane_data.end(),
                   [](const TurnLaneData &data) { return boost::starts_with(data.tag, "merge"); }) +
                   std::size_t{1} >=
               lane_data.size();
    }

    // in case an intersection offers far more lane data items than actual turns, some of them
    // have
    // to be for another intersection. A single additional item can be for an invalid bus lane.
    const auto num_turns = detail::getNumberOfTurns(intersection);

    // more than two additional lane data entries -> lanes target a different intersection
    if (num_turns + std::size_t{2} <= lane_data.size())
    {
        std::cout << "Too mane lane data entries for the current intersection." << std::endl;
        return false;
    }

    // single additional lane data entry is alright, if it is none at the side. This usually
    // refers to a bus-lane
    if (num_turns + std::size_t{1} == lane_data.size())
    {
        std::cout << "Additional Lane Check" << std::endl;
        return lane_data.front().tag == "none" || lane_data.back().tag == "none";
    }

    // more turns than lane data
    if (num_turns > lane_data.size())
    {
        std::cout << "Potentially missing lane entry" << std::endl;
        return lane_data.end() !=
               std::find_if(lane_data.begin(), lane_data.end(), [](const TurnLaneData &data) {
                   return data.tag == "none";
               });
    }

    // find straightmost turn
    const auto straightmost_index = detail::findClosestTurnIndex(intersection, STRAIGHT_ANGLE);
    const auto &straightmost_turn = intersection[straightmost_index];

    // check if we can find a valid 1:1 mapping in a straightforward manner
    bool all_simple = true;
    bool has_none = false;
    std::unordered_set<std::size_t> matched_indices;
    std::cout << "Checking for simple" << std::endl;
    for (const auto &data : lane_data)
    {
        if (data.tag == "none")
        {
            has_none = true;
            continue;
        }
        const auto best_match = detail::findBestMatch(data.tag, intersection);
        std::size_t match_index = std::distance(intersection.begin(), best_match);
        std::cout << "Already Matched? " << matched_indices.count(match_index) << std::endl;
        all_simple &= (matched_indices.count(match_index) == 0);
        matched_indices.insert(match_index);
        std::cout << "Matched: " << data.tag << " to " << toString(*best_match) << std::endl;
        all_simple &= best_match->entry_allowed;
        std::cout << "Allowed: " << best_match->entry_allowed
                  << " Valid: " << detail::isValidMatch(data.tag, best_match->turn.instruction)
                  << std::endl;
        all_simple &= detail::isValidMatch(data.tag, best_match->turn.instruction);
    }

    std::cout << "Check result: " << all_simple << " " << matched_indices.size() << " "
              << lane_data.size() << " " << has_none << std::endl;
    // either all indices are matched, or we have a single none-value
    if (all_simple && (matched_indices.size() == lane_data.size() ||
                       (matched_indices.size() + 1 == lane_data.size() && has_none)))
        return true;

    // better save than sorry
    return false;
}

std::pair<TurnLaneMatcher::LaneDataVector, TurnLaneMatcher::LaneDataVector>
TurnLaneMatcher::partitionLaneData(const NodeID at,
                                   LaneDataVector turn_lane_data,
                                   const Intersection &intersection) const
{
    BOOST_ASSERT(turn_lane_data.size() > detail::getNumberOfTurns(intersection));
    /*
     * A Segregated intersection can provide turn lanes for turns that are not yet possible.
     * The straightforward example would be coming up to the following situation:
     *         (1)             (2)
     *        | A |           | A |
     *        | | |           | ^ |
     *        | v |           | | |
     * -------     -----------     ------
     *  B ->-^                        B
     * -------     -----------     ------
     *  B ->-v                        B
     * -------     -----------     ------
     *        | A |           | A |
     *
     * Traveling on road B, we have to pass A at (1) to turn left onto A at (2). The turn
     * lane itself may only be specified prior to (1) and/or could be repeated between (1)
     * and (2). To make sure to announce the lane correctly, we need to treat the (in this
     * case left) turn lane as if it were to continue straight onto the intersection and
     * look back between (1) and (2) to make sure we find the correct lane for the left-turn.
     */

    // Try and maitch lanes to available turns. For Turns that are not directly matchable, check
    // whether we can match them at the upcoming intersection.

    std::cout << "Trimming Lane Data to Relevant Data" << std::endl;
    for (const auto &road : intersection)
        std::cout << toString(road) << std::endl;
    std::cout << "Lane Data Input:\n";
    for (auto data : turn_lane_data)
        std::cout << "\"" << data.tag << "\" " << (int)data.from << " " << (int)data.to
                  << std::endl;

    const auto straightmos_index = detail::findClosestTurnIndex(intersection, STRAIGHT_ANGLE);
    std::cout << "Index = " << straightmos_index << std::endl;

    const auto &straightmost = intersection[straightmos_index];

    auto straight_tag_index = findTag("through", turn_lane_data);
    // if we have a straight turn, we can check for available turns and postpone other till
    // later
    if (angularDeviation(straightmost.turn.angle, STRAIGHT_ANGLE) < NARROW_TURN_ANGLE &&
        straight_tag_index < turn_lane_data.size())
    {
        std::cout << "Next Intersection: " << at << " eid: " << straightmost.turn.eid << std::endl;
        auto next_intersection = turn_analysis.getIntersection(at, straightmost.turn.eid);
        next_intersection =
            turn_analysis.assignTurnTypes(at, straightmost.turn.eid, std::move(next_intersection));

        std::cout << "Next Intersection\n";
        for (auto road : next_intersection)
            std::cout << toString(road) << std::endl;

        BOOST_ASSERT(detail::isValidMatch(
            turn_lane_data[straight_tag_index].tag,
            detail::findBestMatch(turn_lane_data[straight_tag_index].tag, intersection)
                ->turn.instruction));

        bool can_match_all_items = true;
        auto separator = std::partition(
            turn_lane_data.begin(), turn_lane_data.end(), [&](const TurnLaneData &lane_data) {
                auto best_match = detail::findBestMatch(lane_data.tag, intersection);
                if (detail::isValidMatch(lane_data.tag, best_match->turn.instruction))
                {
                    std::cout << "Can Match: " << lane_data.tag
                              << " to id: " << std::distance(intersection.begin(), best_match)
                              << std::endl;
                    return true;
                }
                else
                {
                    std::cout << "Straightmost: " << straightmost.turn.angle
                              << " starting at: " << at << " via " << straightmost.turn.eid
                              << " results in Intersection:" << std::endl;
                    for (const auto &road : next_intersection)
                        std::cout << toString(road) << std::endl;
                    auto best_match_at_next_intersection =
                        detail::findBestMatch(lane_data.tag, next_intersection);
                    if (detail::isValidMatch(lane_data.tag,
                                             best_match_at_next_intersection->turn.instruction))
                    {
                        std::cout << "Can Match At Next Intersection -> Erase and Augment Through"
                                  << std::endl;
                    }
                    else
                    {
                        can_match_all_items = false;
                    }
                    return false;
                }

            });
        if (can_match_all_items)
        {
            // augment the straight turn to
            auto &straight_item = turn_lane_data[findTag("through", turn_lane_data)];
            auto straight_copy = straight_item;
            auto offset = std::distance(turn_lane_data.begin(), separator);
            for (auto itr = separator; itr != turn_lane_data.end(); ++itr)
            {
                // TODO we should check whether these are actually next to each other.
                straight_item.from = std::min(straight_item.from, itr->from);
                straight_item.to = std::max(straight_item.to, itr->to);
            }

            std::sort(turn_lane_data.begin(), separator);
            turn_lane_data.push_back(straight_copy);
            std::sort(turn_lane_data.begin() + offset, turn_lane_data.end());
            std::cout << "Lane Data Output:\n";
            for (auto data : turn_lane_data)
                std::cout << "\"" << data.tag << "\" " << (int)data.from << " " << (int)data.to
                          << std::endl;
            return {{turn_lane_data.begin(), turn_lane_data.begin() + offset},
                    {turn_lane_data.begin() + offset, turn_lane_data.end()}};
        }
        else
        {
            std::cout << "Failed to match all items" << std::endl;
            // Well, we messed up
            std::sort(turn_lane_data.begin(), turn_lane_data.end());
        }
    }
    return {std::move(turn_lane_data), {}};
}

Intersection TurnLaneMatcher::simpleMatchTuplesToTurns(Intersection intersection,
                                                       const LaneID num_lanes,
                                                       const LaneDataVector &lane_data) const
{
    const auto possible_entries =
        std::count_if(intersection.begin(), intersection.end(), [](const ConnectedRoad &road) {
            return road.entry_allowed;
        });

    for (auto entry : lane_data)
        if (entry.tag == "none")
            return intersection;

    // Needs to handle u-turn edge, sort lanes accordingly
    std::cout << "Lane Data: " << lane_data.size() << " "
              << "Intersection: " << intersection.size() << " Entries: " << possible_entries
              << std::endl;
    BOOST_ASSERT(lane_data.size() + (intersection[0].entry_allowed ? 1 : 0) == possible_entries);

    std::cout << "Lane Data Input:\n";
    for (auto data : lane_data)
        std::cout << "\"" << data.tag << "\" " << (int)data.from << " " << (int)data.to
                  << std::endl;
    std::cout << "Intersection:\n";
    for (auto &road : intersection)
        std::cout << "\t" << toString(road) << std::endl;

    for (std::size_t road_index = 1, valid_turn = 0;
         road_index < intersection.size() && valid_turn < possible_entries;
         ++road_index)
    {
        if (intersection[road_index].entry_allowed)
        {
            BOOST_ASSERT(lane_data[valid_turn].from != INVALID_LANEID);
            intersection[road_index].turn.instruction.lane_tupel = {
                LaneID(lane_data[valid_turn].to - lane_data[valid_turn].from + 1),
                lane_data[valid_turn].from};
            const bool uses_all_lanes =
                lane_data[valid_turn].to - lane_data[valid_turn].from + 1 == num_lanes;
            if (TurnType::Suppressed == intersection[road_index].turn.instruction.type &&
                !uses_all_lanes)
                intersection[road_index].turn.instruction.type = TurnType::UseLane;
            std::cout << "Assigned: " << lane_data[valid_turn].tag << " to "
                      << toString(intersection[road_index]) << std::endl;
            BOOST_ASSERT(detail::findBestMatch(lane_data[valid_turn].tag, intersection) ==
                         intersection.begin() + road_index);
            ++valid_turn;
        }
    }
    return intersection;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
