#include "GPS_Path.h"

GPS_Path::GPS_Path(const std::string &f_p, const std::string &reading_file_name, double m_l, double curve_R, double r_r = 10,
                   int step_in_miters_ = 1000) : step_in_miters(step_in_miters_), files_path(f_p), min_dist(m_l), min_radius_of_curvature(curve_R), radius_of_reaching(r_r) {
    logs.start_process("INIT GPS_Path");

    std::ifstream file(files_path + reading_file_name);
    std::string line;
    getline(file, line);
    while (getline(file, line)) {
        double x, y, h;
        std::istringstream(line) >> x >> y >> h;
        control_points.emplace_back(Vec3D{x, y, h});
    }
    file.close();

    logs.start_process("LINE BUILDING");
    time_t now = time(0);
    char *t = ctime(&now);

    path_line = CatmullROM{control_points, files_path + add_time("path") + ".txt",
                               min_radius_of_curvature, float(step_in_miters), }.get_path();
    logs.finish_process();
    logs.finish_process();
};

Vec3D GPS_Path::get_next_point(Vec3D curr_pos, bool need_to_set = 1) {
    if (last_point_id >= path_line.size())
      return Vec3D {-1, -1, -1};
    int p_id = last_point_id + 1;
    while (path_line[p_id].lengh(curr_pos) < min_dist)
    {
        p_id++;
        if (p_id >= path_line.size())
          return path_line[path_line.size() - 1];
    }
    if (need_to_set)
        target_point_id = p_id;
    return path_line[p_id];
}

Vec3D GPS_Path::get_point_by_dist(int m, bool need_to_set = 0) {
    int p_id = (m + step_in_miters - 1) / step_in_miters;
    if (need_to_set)
        target_point_id = p_id;
    return control_points[p_id];
}
