#ifndef AVES_LINES_H
#define AVES_LINES_H

#include "libs.h"
#include "structs.h"
#include "funcs.h"
extern Log logs;

//base calass for trajectory construction classes
class Line {
protected:
    double path_step; // длина шага цепи в метрах
    std::string files_addres;
    std::vector<Vec3D> cruve;
    std::vector<Vec3D> points;

public:
    std::vector<Vec3D> get_path() {
        return points;
    }

    virtual void build() {}
};

//calass for CatmullROM-curve-trajectory building
class CatmullROM : public Line {
public:
    CatmullROM(const std::vector<Vec3D> &points_, std::string f_add, double min_R_, double ps = 10)
    : start_p(points_[0]), end_p(points_[points_.size() -1]), min_R(min_R_) {
        points = points_;
        files_addres = std::move(f_add);
        path_step = ps;

        if (points_.size() < 2) {
            throw "InitError: at least 2 points are required to construct the curve";
        }
        logs.start_process("CATMULLROM BUILDING");

        build();
        for (auto & point : points) {
            logs.add_point(point.x, point.y);
        }
        logs.finish_process();
    }

    CatmullROM() = default;

private:
    Vec3D start_p, end_p;
    double min_R;

    void build() override;
    
    static void get_bis_coef_and_circ_center(std::vector<Vec3D>& points, int i, double min_R_in_d_lat, double& bis_k, double& bis_b, double& x0, double& y0);

    static void get_touch_points(std::vector<Vec3D>& points, int j, double x0, double y0, double min_R_in_d_lat, Vec2D& touch_point1, Vec2D& touch_point2);

    static void check_order(std::vector<Vec3D>& points_full);

    void add_first_touch_point(std::vector<Vec3D>& points_full, Vec2D touch_point1, Vec2D touch_point2, int i);

    void add_second_touch_point(std::vector<Vec3D>& points_full, Vec2D touch_point1, Vec2D touch_point2, int i, double bis_k, double bis_b);
};

#endif //AVES_LINES_H
