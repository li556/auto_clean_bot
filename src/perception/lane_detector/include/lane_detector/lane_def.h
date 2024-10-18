#pragma once

// 车道线相较于车辆的位置类型
enum LaneLinePositionType {
    CURB_LEFT = -5,
    FOURTH_LEFT = -4,
    THIRD_LEFT = -3,
    ADJACENT_LEFT = -2, //!< lane marking on the left side next to ego lane
    EGO_LEFT = -1,      //!< left lane marking of the ego lane
    EGO_CENTER = 0,     //!< center lane marking of the ego lane, changing lane
    EGO_RIGHT = 1,      //!< right lane marking of the ego lane
    ADJACENT_RIGHT = 2, //!< lane marking on the right side next to ego lane
    THIRD_RIGHT = 3,
    FOURTH_RIGHT = 4,
    CURB_RIGHT = 5,
    OTHER = 6,  //!< other types of lane
    UNKNOWN = 7 //!< background
};

// 车道线类型
enum LaneLineType { WHITE_DASHED = 0, WHITE_SOLID, YELLOW_DASHED, YELLOW_SOLID };