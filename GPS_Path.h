#ifndef AVES_GPS_PATH_H
#define AVES_GPS_PATH_H

#include "libs.h"
#include "funcs.h"
#include "structs.h"
#include "Lines.h"

//class for building a path based on gps points
class GPS_Path {
private:
    std::vector<Vec3D> control_points;
    std::vector<Vec3D> path_line;
    std::string files_path;
    int step_in_miters;
    int last_point_id = -1; //номер последней посещённей точки
    int target_point_id = 0; //номер текущей целевой точки
    double radius_of_reaching; // см check_if_pos_in_point(Vec3D)
    double min_dist; // при запросе get_next_point выдаётся точка, принадлежащая к траектории, находящаяся на расстоянии не меньше min_dist
    double min_radius_of_curvature; // минимальный радиус кривизны траектории в метрах
    // (! в вычислениях переводится в дельту по широте, что приводит к погрешности
    // (в пределах Жуковского незначитальная, но точная величина не известна))

public:
    GPS_Path(const std::string &, const std::string &, double, double, double, int);

    //returns a list of three-dimensional points that describe the trajectory, compiled about the reference points
    std::vector<Vec3D> get_path() { return path_line; }

    //returns such point, that the length of the path to it is the smallest, but not less than [m]
    Vec3D get_point_by_dist(int, bool);

    //returns such point:
    // 1) that located at the distance of at least min_lenght from aircraft current position
    // 2) that has a greater id than last visited point id
    Vec3D get_next_point(Vec3D, bool);

    Vec3D get_target_point() { return path_line[target_point_id]; }

    //по переданной позиции определяет, достигнута ли текущая целевая точка
    bool check_if_pos_in_point(Vec3D position) {
        if (path_line[target_point_id].lengh(position) < radius_of_reaching){
            last_point_id = target_point_id;
            get_next_point(position, 1); 
            return true;
        }
        return false;
    }
    
    void set_target_pos_as_achieved() {last_point_id = target_point_id;} 
};

#endif //AVES_GPS_PATH_H
