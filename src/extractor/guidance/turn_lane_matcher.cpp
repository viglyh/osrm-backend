#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/turn_lane_matcher.hpp"
#include "util/simple_logger.hpp"
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
                                               "merge_to_left",
                                               "merge_to_right"};
    const auto index =
        std::distance(tag_by_modifier, std::find(tag_by_modifier, tag_by_modifier + 10, tag));

    BOOST_ASSERT(index <= 10);

    const constexpr DirectionModifier::Enum modifiers[11] = {
        DirectionModifier::UTurn,
        DirectionModifier::SharpRight,
        DirectionModifier::Right,
        DirectionModifier::SlightRight,
        DirectionModifier::Straight,
        DirectionModifier::SlightLeft,
        DirectionModifier::Left,
        DirectionModifier::SharpLeft,
        DirectionModifier::Straight,
        DirectionModifier::Straight,
        DirectionModifier::UTurn}; // fallback for invalid tags

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

bool isValidMatch(const std::string &tag, const TurnInstruction instruction)
{
    const auto hasLeftModifier = [](const TurnInstruction instruction) {
        return instruction.direction_modifier == DirectionModifier::SlightLeft ||
               instruction.direction_modifier == DirectionModifier::Left ||
               instruction.direction_modifier == DirectionModifier::SharpLeft;
    };

    const auto hasRightModifier = [](const TurnInstruction instruction) {
        return instruction.direction_modifier == DirectionModifier::SlightRight ||
               instruction.direction_modifier == DirectionModifier::Right ||
               instruction.direction_modifier == DirectionModifier::SharpRight;
    };

    const auto isMirroredModifier = [](const TurnInstruction instruction) {
        return instruction.type == TurnType::Merge;
    };

    if (tag == "reverse")
    {
        return hasLeftModifier(instruction) ||
               instruction.direction_modifier == DirectionModifier::UTurn;
    }
    else if (tag == "sharp_right" || tag == "right" || tag == "slight_right")
    {
        if (isMirroredModifier(instruction))
            return hasLeftModifier(instruction);
        else
            return hasRightModifier(instruction);
    }
    else if (tag == "through")
    {
        return instruction.direction_modifier == DirectionModifier::Straight ||
               instruction.type == TurnType::Suppressed || instruction.type == TurnType::NewName ||
               instruction.type == TurnType::StayOnRoundabout || entersRoundabout(instruction) ||
               (instruction.type ==
                    TurnType::Fork && // Forks can be experienced, even for straight segments
                (instruction.direction_modifier == DirectionModifier::SlightLeft ||
                 instruction.direction_modifier == DirectionModifier::SlightRight)) ||
               (instruction.type ==
                    TurnType::Continue && // Forks can be experienced, even for straight segments
                (instruction.direction_modifier == DirectionModifier::SlightLeft ||
                 instruction.direction_modifier == DirectionModifier::SlightRight)) ||
               instruction.type == TurnType::UseLane;
    }
    else if (tag == "slight_left" || tag == "left" || tag == "sharp_left")
    {
        if (isMirroredModifier(instruction))
            return hasRightModifier(instruction);
        else
            return hasLeftModifier(instruction);
    }
    return false;
}

typename Intersection::const_iterator findBestMatch(const std::string &tag,
                                                    const Intersection &intersection)
{
    const constexpr double idealized_turn_angles[] = {0, 35, 90, 135, 180, 225, 270, 315};
    const auto idealized_angle = idealized_turn_angles[getMatchingModifier(tag)];
    return std::min_element(
        intersection.begin(),
        intersection.end(),
        [idealized_angle, &tag](const ConnectedRoad &lhs, const ConnectedRoad &rhs) {
            // prefer valid matches
            if (isValidMatch(tag, lhs.turn.instruction) != isValidMatch(tag, rhs.turn.instruction))
                return isValidMatch(tag, lhs.turn.instruction);
            // if the entry allowed flags don't match, we select the one with
            // entry allowed set to true
            if (lhs.entry_allowed != rhs.entry_allowed)
                return lhs.entry_allowed;

            return angularDeviation(idealized_angle, lhs.turn.angle) <
                   angularDeviation(idealized_angle, rhs.turn.angle);
        });
}

bool canMatchTrivially(const Intersection &intersection,
                       const TurnLaneMatcher::LaneDataVector &lane_data)
{
    std::size_t road_index = 1, lane = 0;
    for (; road_index < intersection.size() && lane < lane_data.size(); ++road_index)
    {
        if (intersection[road_index].entry_allowed)
        {
            BOOST_ASSERT(lane_data[lane].from != INVALID_LANEID);
            if (!isValidMatch(lane_data[lane].tag, intersection[road_index].turn.instruction))
                return false;

            if (detail::findBestMatch(lane_data[lane].tag, intersection) !=
                intersection.begin() + road_index)
                return false;
            ++lane;
        }
    }

    // TODO handle reverse
    return lane == lane_data.size() ||
           (lane + 1 == lane_data.size() && lane_data.back().tag == "reverse");
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
        data.lane_string_id != INVALID_LANE_STRINGID ? turn_lane_strings.GetNameForID(data.lane_string_id) : "";
    // FIXME this is a workaround due to https://github.com/cucumber/cucumber-js/issues/417,
    // need to switch statements when fixed
    // const auto num_lanes = std::count(turn_lane_string.begin(), turn_lane_string.end(), '|') + 1;

    // going straight, due to traffic signals, we can have uncompressed geometry
    if (intersection.size() == 2 &&
        (data.lane_string_id == node_based_graph.GetEdgeData(intersection[1].turn.eid).lane_string_id ||
         intersection[1].turn.instruction.type == TurnType::Suppressed))
        return intersection;

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
        LaneDataVector lane_data;
        if (lane_string.empty())
            return lane_data;

        do
        {
            // FIXME this is a cucumber workaround, since escaping does not work properly in
            // cucumber.js (see https://github.com/cucumber/cucumber-js/issues/417). Needs to be
            // changed to "|" only, when the bug is fixed
            auto lane = getNextTag(lane_string, "|&");
            setLaneData(lane_map, lane, lane_nr);
            ++lane_nr;
        } while (lane_nr < num_lanes);

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
        auto previous_string = previous_data.lane_string_id != INVALID_LANE_STRINGID
                                   ? turn_lane_strings.GetNameForID(previous_data.lane_string_id)
                                   : "";

        if (previous_string.empty())
            return previous_string;

        previous_intersection =
            turn_analysis.assignTurnTypes(node_x, previous_id, std::move(previous_intersection));

        auto previous_lane_data =
            convertLaneStringToData(countLanes(previous_string), previous_string);
        if (isSimpleIntersection(previous_lane_data, previous_intersection))
            return "";

        return previous_string;
    }();

    if (turn_lane_string.empty() && previous_lane_string.empty())
    {
        for (auto &road : intersection)
            road.turn.instruction.lane_tupel = {0, INVALID_LANEID};

        return intersection;
    }
    LaneID num_lanes = countLanes(turn_lane_string);

    auto lane_data = convertLaneStringToData(num_lanes, turn_lane_string);
    std::cout << "Lane: " << turn_lane_string << " Previous: " << previous_lane_string << std::endl;
    std::cout << "Intersection:\n";
    for (auto road : intersection)
        std::cout << "\t" << toString(road) << std::endl;

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
    const std::size_t possible_entries =
        std::count_if(intersection.begin(), intersection.end(), [](const ConnectedRoad &road) {
            return road.entry_allowed;
        });

    if (is_simple && !lane_data.empty() &&
        lane_data.size() !=
            static_cast<std::size_t>(lane_data.back().tag == "reverse" ? 1 : 0) +
                possible_entries &&
        intersection[0].entry_allowed && lane_data.size() == findTag("none", lane_data))
    {
        std::cout << "Adding entry to lane data" << std::endl;
        // Needs to handle u-turn edge, sort lanes accordingly
        if (lane_data.back().tag != "left" && lane_data.back().tag != "sharp_left")
        {
            std::cout << "ADD" << std::endl;
            auto at = node_based_graph.GetTarget(via_edge);
            std::cout << "Location: " << std::setprecision(12) << toFloating(node_info_list[at].lat)
                      << " " << toFloating(node_info_list[at].lon) << std::endl;
        }
        BOOST_ASSERT(lane_data.back().tag == "left" || lane_data.back().tag == "sharp_left");
        lane_data.push_back({"reverse", lane_data.back().to, lane_data.back().to});
    }

    std::cout << "IsSimple: " << is_simple << std::endl;

    if (is_simple && !turn_lane_string.empty())
    {
        lane_data = handleNoneValueAtSimpleTurn(std::move(lane_data), intersection);
        std::cout << "A" << std::endl;
        if (detail::canMatchTrivially(intersection, lane_data))
            return simpleMatchTuplesToTurns(std::move(intersection), num_lanes, lane_data);
        else
            return intersection;
    }
    else if (turn_lane_string.empty() && !previous_lane_string.empty())
    {
        num_lanes = countLanes(previous_lane_string);
        lane_data = convertLaneStringToData(num_lanes, previous_lane_string);
        is_simple = isSimpleIntersection(lane_data, intersection);

        if (is_simple && !previous_lane_string.empty())
        {
            lane_data = handleNoneValueAtSimpleTurn(std::move(lane_data), intersection);
            std::cout << "B" << std::endl;
            if (detail::canMatchTrivially(intersection, lane_data))
                return simpleMatchTuplesToTurns(std::move(intersection), num_lanes, lane_data);
            else
                return intersection;
        }
        else if (!previous_lane_string.empty())
        {
            if (lane_data.size() >= detail::getNumberOfTurns(previous_intersection) &&
                previous_intersection.size() != 2)
            {
                std::cout << "CP" << std::endl;
                auto at = node_based_graph.GetTarget(via_edge);
                std::cout << "Location: " << std::setprecision(12)
                          << toFloating(node_info_list[at].lat) << " "
                          << toFloating(node_info_list[at].lon) << std::endl;
                lane_data = partitionLaneData(node_based_graph.GetTarget(previous_id),
                                              std::move(lane_data),
                                              previous_intersection)
                                .second;

                std::sort(lane_data.begin(), lane_data.end());

                // check if we were successfull in trimming
                if (lane_data.size() == detail::getNumberOfTurns(intersection) &&
                    isSimpleIntersection(lane_data, intersection))
                {
                    lane_data = handleNoneValueAtSimpleTurn(std::move(lane_data), intersection);

                    std::cout << "C" << std::endl;

                    if (detail::canMatchTrivially(intersection, lane_data))
                        return simpleMatchTuplesToTurns(
                            std::move(intersection), num_lanes, lane_data);
                    else
                        return intersection;
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
        if (lane_data.size() >= detail::getNumberOfTurns(intersection))
        {
            std::cout << "DP" << std::endl;
            if (findTag("merge_to_left", lane_data) != lane_data.size() ||
                findTag("merge_to_right", lane_data) != lane_data.size())
                return intersection;

            lane_data = partitionLaneData(node_based_graph.GetTarget(via_edge),
                                          std::move(lane_data),
                                          intersection)
                            .first;

            // check if we were successfull in trimming
            if (lane_data.size() == detail::getNumberOfTurns(intersection) &&
                isSimpleIntersection(lane_data, intersection))
            {
                lane_data = handleNoneValueAtSimpleTurn(std::move(lane_data), intersection);

                std::cout << "D" << std::endl;

                if (detail::canMatchTrivially(intersection, lane_data))
                    return simpleMatchTuplesToTurns(std::move(intersection), num_lanes, lane_data);
                else
                    return intersection;
            }
        }
    }
    for (auto &road : intersection)
        road.turn.instruction.lane_tupel = {0, INVALID_LANEID};

    return intersection;
}

/*
    Lanes can have the tag none. While a nice feature for visibility, it is a terrible feature
   for
   parsing. None can be part of neighboring turns, or not. We have to look at both the
   intersection
   and the lane data to see what turns we have to augment by the none-lanes
 */
TurnLaneMatcher::LaneDataVector
TurnLaneMatcher::handleNoneValueAtSimpleTurn(LaneDataVector lane_data,
                                             const Intersection &intersection) const
{
    if (intersection.empty() || lane_data.empty() || findTag("none", lane_data) == lane_data.size())
    {
        return lane_data;
    }

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

    if (intersection[0].entry_allowed && lane_data.back().tag != "reverse")
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
                    util::SimpleLogger().Write(logWARNING)
                        << "Failed lane assignment. Reached bad situation.";
                    return std::make_pair(std::size_t{0}, std::size_t{0});
                }();
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
                    }
                }
                lane_data.erase(lane_data.begin() + index);
                std::sort(lane_data.begin(), lane_data.end());
            }
            // we have to reduce it, assigning it to neighboring turns
            else if (connection_count < lane_data.size())
            {
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
    if (lane_data.empty())
        return true;
    // if we are on a straight road, turn lanes are only reasonable in connection to the next
    // intersection, or in case of a merge. If not all but one (straight) are merges, we don't
    // consider the intersection simple
    if (intersection.size() == 2)
    {
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
    const auto num_turns = detail::getNumberOfTurns(intersection) +
                           (!intersection[0].entry_allowed && lane_data.back().tag == "reverse");

    // more than two additional lane data entries -> lanes target a different intersection
    if (num_turns + std::size_t{2} <= lane_data.size())
    {
        return false;
    }

    // single additional lane data entry is alright, if it is none at the side. This usually
    // refers to a bus-lane
    if (num_turns + std::size_t{1} == lane_data.size() && lane_data.front().tag != "none" &&
        lane_data.back().tag != "none")
        return false;

    // more turns than lane data
    if (num_turns > lane_data.size() &&
        lane_data.end() ==
            std::find_if(lane_data.begin(), lane_data.end(), [](const TurnLaneData &data) {
                return data.tag == "none";
            }))
        return false;

    if (num_turns > lane_data.size() && intersection[0].entry_allowed &&
        !(findTag("reverse", lane_data) ||
          (lane_data.back().tag != "left" && lane_data.back().tag != "sharp_left")))
        return false;

    // check if we can find a valid 1:1 mapping in a straightforward manner
    bool all_simple = true;
    bool has_none = false;
    std::unordered_set<std::size_t> matched_indices;
    for (const auto &data : lane_data)
    {
        if (data.tag == "none")
        {
            has_none = true;
            continue;
        }

        const auto best_match = detail::findBestMatch(data.tag, intersection);
        std::size_t match_index = std::distance(intersection.begin(), best_match);
        all_simple &= (matched_indices.count(match_index) == 0);
        matched_indices.insert(match_index);
        // in case of u-turns, we might need to activate them first
        all_simple &= (best_match->entry_allowed || match_index == 0);
        all_simple &= detail::isValidMatch(data.tag, best_match->turn.instruction);
        std::cout << "Tag: " << data.tag << " matched to: " << match_index << " valid: " << detail::isValidMatch(data.tag,best_match->turn.instruction) << " all simple now: " << all_simple << std::endl;
    }

    std::cout << "All Simple: " << all_simple << std::endl;
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
    BOOST_ASSERT(turn_lane_data.size() >= detail::getNumberOfTurns(intersection));
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

    const auto straightmost_index = detail::findClosestTurnIndex(intersection, STRAIGHT_ANGLE);

    BOOST_ASSERT(straightmost_index < intersection.size());
    const auto &straightmost = intersection[straightmost_index];

    // if we have a straight turn, we can check for available turns and postpone other till
    // later
    if (angularDeviation(straightmost.turn.angle, STRAIGHT_ANGLE) < FUZZY_ANGLE_DIFFERENCE)
    {
        auto straight_tag_index = findTag("through", turn_lane_data);
        bool added = false;
        if (straight_tag_index == turn_lane_data.size() && straightmost.entry_allowed &&
            findTag("slight_left", turn_lane_data) == turn_lane_data.size() &&
            findTag("slight_right", turn_lane_data) == turn_lane_data.size())
        {
            std::cout << "Added Through" << std::endl;
            turn_lane_data.push_back({"through",
                                      std::numeric_limits<LaneID>::max(),
                                      std::numeric_limits<LaneID>::min()});
            added = true;
        }

        auto next_intersection = turn_analysis.getIntersection(at, straightmost.turn.eid);
        next_intersection =
            turn_analysis.assignTurnTypes(at, straightmost.turn.eid, std::move(next_intersection));

        if (straight_tag_index != turn_lane_data.size() &&
            !detail::isValidMatch(
                turn_lane_data[straight_tag_index].tag,
                detail::findBestMatch(turn_lane_data[straight_tag_index].tag, intersection)
                    ->turn.instruction))
        {
            std::cout << "Failed to match: " << turn_lane_data[straight_tag_index].tag << std::endl;
            std::cout << "Best Match: "
                      << detail::findBestMatch(turn_lane_data[straight_tag_index].tag, intersection)
                             ->turn.angle
                      << std::endl;
            std::cout << "Location: " << std::setprecision(12) << toFloating(node_info_list[at].lat)
                      << " " << toFloating(node_info_list[at].lon) << std::endl;
            std::cout << "Turns\n";
            for (auto road : intersection)
                std::cout << "\t" << toString(road) << std::endl;
        }
        bool can_match_all_items = false;
        if ((turn_lane_data.size() == straight_tag_index ||
             detail::isValidMatch(
                 turn_lane_data[straight_tag_index].tag,
                 detail::findBestMatch(turn_lane_data[straight_tag_index].tag, intersection)
                     ->turn.instruction)))
        {
            can_match_all_items = true; // be optimistic to begin with
            auto separator = std::partition(
                turn_lane_data.begin(), turn_lane_data.end(), [&](const TurnLaneData &lane_data) {
                    auto best_match = detail::findBestMatch(lane_data.tag, intersection);
                    if (detail::isValidMatch(lane_data.tag, best_match->turn.instruction) ||
                        (lane_data.tag == "reverse" && best_match == intersection.begin()))
                    {
                        return true;
                    }
                    else
                    {
                        auto best_match_at_next_intersection =
                            detail::findBestMatch(lane_data.tag, next_intersection);
                        if (!detail::isValidMatch(
                                lane_data.tag, best_match_at_next_intersection->turn.instruction))
                        {
                            can_match_all_items = false;
                        }
                        return false;
                    }

                });

            if (can_match_all_items && findTag("through", turn_lane_data) != turn_lane_data.size())
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
                std::cout << "Modified Straight to: " << straight_item.from << " "
                          << straight_item.to << std::endl;

                std::sort(turn_lane_data.begin(), separator);
                if (!added)
                    turn_lane_data.push_back(straight_copy);
                std::sort(turn_lane_data.begin() + offset, turn_lane_data.end());
                return {{turn_lane_data.begin(), turn_lane_data.begin() + offset},
                        {turn_lane_data.begin() + offset, turn_lane_data.end()}};
            }
        }
        std::cout << "Coudln't match all" << std::endl;
        if (added)
        {
            turn_lane_data.erase(turn_lane_data.begin() + findTag("through", turn_lane_data));
        }
        // Well, we messed up
        std::sort(turn_lane_data.begin(), turn_lane_data.end());
    }
    return {std::move(turn_lane_data), {}};
}

Intersection TurnLaneMatcher::simpleMatchTuplesToTurns(Intersection intersection,
                                                       const LaneID num_lanes,
                                                       const LaneDataVector &lane_data) const
{
    std::cout << "Assign Turns" << std::endl;
    if (lane_data.empty())
        return intersection;

    if (intersection.size() == 2 ||
        std::count_if(lane_data.begin(), lane_data.end(), [](const TurnLaneData &data) {
            return boost::starts_with(data.tag, "merge");
        }) > 0)
    {
        std::cout << "Bailing since intersection size is low, or we consider merge" << std::endl;
        return intersection;
    }

    const std::size_t possible_entries =
        std::count_if(intersection.begin(), intersection.end(), [](const ConnectedRoad &road) {
            return road.entry_allowed;
        });

    for (auto entry : lane_data)
        if (entry.tag == "none")
        {
            util::SimpleLogger().Write(logDEBUG)
                << "Did not handle \"none\" prior to lane assignment";
            return intersection;
        }

    /*
    if (!(lane_data.size() ==
          static_cast<std::size_t>(
              (lane_data.back().tag == "reverse" && !intersection[0].entry_allowed) ? 1 : 0) +
              possible_entries))
    {
        std::cout << "Data: " << lane_data.size() << " entries: " << possible_entries << std::endl;
    }
    BOOST_ASSERT(
        lane_data.size() ==
        static_cast<std::size_t>(
            (lane_data.back().tag == "reverse" && (!intersection[0].entry_allowed &&
    findTag("sharp_left",lane_data) == lane_data.size()  ) ? 1 : 0) +
            possible_entries);
    */

    std::size_t valid_turn = 0;
    for (std::size_t road_index = 1;
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
            if (detail::findBestMatch(lane_data[valid_turn].tag, intersection) !=
                intersection.begin() + road_index)
            {
                std::cout << "Failed to match: " << lane_data[valid_turn].tag << std::endl;
                std::cout
                    << "Best Match: "
                    << detail::findBestMatch(lane_data[valid_turn].tag, intersection)->turn.angle
                    << std::endl;
                std::cout
                    << "Location: " << std::setprecision(12)
                    << toFloating(
                           node_info_list[node_based_graph.GetTarget(intersection[0].turn.eid)].lat)
                    << " "
                    << toFloating(
                           node_info_list[node_based_graph.GetTarget(intersection[0].turn.eid)].lon)
                    << std::endl;
                for (auto ld : lane_data)
                    std::cout << "\t" << ld.tag << " " << (int)ld.from << " " << (int)ld.to
                              << std::endl;
                std::cout << "Turns\n";
                for (auto road : intersection)
                    std::cout << "\t" << toString(road) << std::endl;
            }
            BOOST_ASSERT(detail::findBestMatch(lane_data[valid_turn].tag, intersection) ==
                         intersection.begin() + road_index);
            ++valid_turn;
            if (TurnType::Suppressed == intersection[road_index].turn.instruction.type &&
                !uses_all_lanes)
                intersection[road_index].turn.instruction.type = TurnType::UseLane;
        }
    }

    if (valid_turn < lane_data.size())
    {
        // TODO make sure that we are not at an intersection with a middle island
        if (lane_data.back().tag == "reverse" && valid_turn + 1 == lane_data.size())
        {
            std::size_t u_turn = 0;
            if (node_based_graph.GetEdgeData(intersection[0].turn.eid).reversed)
            {
                if (intersection.back().entry_allowed ||
                    intersection.back().turn.instruction.direction_modifier !=
                        DirectionModifier::SharpLeft)
                    return intersection;
                u_turn = intersection.size() - 1;
            }
            intersection[u_turn].entry_allowed = true;
            intersection[u_turn].turn.instruction.type = TurnType::Turn;
            intersection[u_turn].turn.instruction.direction_modifier = DirectionModifier::UTurn;
            intersection[u_turn].turn.instruction.lane_tupel = {
                LaneID(lane_data.back().to - lane_data.back().from + 1), lane_data.back().from};
        }
        else
        {
            util::SimpleLogger().Write(logDEBUG) << "failed in turn lane matching.";
        }
    }

    std::cout << "Successfully assigned turns" << std::endl;
    return intersection;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
