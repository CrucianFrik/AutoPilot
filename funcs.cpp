#include "funcs.h"

//add current date and time to string as a start
std::string add_time(std::string s) {
    time_t now = time(0);
    std::string t = ctime(&now);
    return t + ' ' + s;
}

//calculate dh and dFi
ErrorPair calc_errors(Vec3D target_point, Vec3D curr_pos, Vec3D prev_pos) {
    Vec2D curr_tar{target_point.x - curr_pos.x, target_point.y - curr_pos.y};
    Vec2D speed_vec{ curr_pos.x - prev_pos.x, curr_pos.y - prev_pos.y };
    double Fi = acos((curr_tar.x * speed_vec.x + curr_tar.y * speed_vec.y) /
                     (curr_tar.len() * speed_vec.len()));
    return ErrorPair{target_point.h - curr_pos.h,
                     Fi - speed_vec.x}; //in a connected frame of reference, the x-axis runs from the tail to the nose
}