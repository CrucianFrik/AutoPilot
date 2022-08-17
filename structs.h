#ifndef AVES_STRUCTS_H
#define AVES_STRUCTS_H

#include "libs.h"
#include <ctime>

enum PathType {
    curve,
    polyline
};

//struct for calculating cubic polynomes (is used in CatmullROM building)
struct CubicPoly {
    double c0, c1, c2, c3;

    double eval(double t) const {
        double t2 = t * t;
        double t3 = t2 * t;
        return c0 + c1 * t + c2 * t2 + c3 * t3;
    }
};

struct Vec2D {
    double x, y; ///???? 2d 3d
    double len() { return sqrt(x * x + y * y); }

    Vec2D operator*(double d) {
        return Vec2D{x * d, y * d};
    }

    Vec2D operator/(double d) {
        return Vec2D{x / d, y / d};
    }

    Vec2D operator+(Vec2D v)
    {
        return Vec2D{x+v.x, y+v.y};
    }

    Vec2D operator-(Vec2D v)
    {
        return Vec2D{x-v.x, y-v.y};
    }

    bool operator== (Vec2D v)
    {
        if (v.x == x && v.y == y)
            return true;
        return false;
    }

    double lengh(Vec2D vec) const {
        double degree_to_rad = double(M_PI / 180);

        double d_lat = (x - vec.x) * degree_to_rad;
        double d_long = (y - vec.y) * degree_to_rad;

        double a =
                pow(sin(d_lat / 2), 2) + cos(x * degree_to_rad) * cos(vec.x * degree_to_rad) * pow(sin(d_long / 2), 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        double merid_circ_len = 6367;
        double km_to_m = 1000;
        double m = merid_circ_len * c * km_to_m;
        return m;
    }

    double angel(Vec2D vec){
        return acos((x*vec.x + y*vec.y) / (len()*vec.len()));
    }
};


//struct for working with a three-dimensional point
struct Vec3D {
    double x, y, h;

    Vec3D(double _x, double _y, double _h) : x(_x), y(_y), h(_h) {}

    Vec3D(Vec2D vec, double _h) : x(vec.x), y(vec.y), h(_h) {}

    Vec3D() {}

    static double similar_digit(double d) { return d * 0.99999 + 0.0001; }

    //(is used in creating the start- and end-points in CatmullROM building)
    Vec3D make_similar() const {
        return Vec3D{similar_digit(x), similar_digit(y), similar_digit(h)};
    }

    std::string get_str() { return std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(h) + "\n"; }

    double lengh(Vec3D vec) const {
        return sqrt(pow(Vec2D{x, y}.lengh(Vec2D{vec.x, vec.y}), 2)+pow(h - vec.h, 2));
    }

    Vec2D get_2d()
    {
        return Vec2D {x, y};
    }

    Vec3D operator+ (Vec3D v)
    {
        return Vec3D{x+v.x, y+v.y, h+v.h};
    }

    Vec3D operator- (Vec3D v)
    {
        return Vec3D{x-v.x, y-v.y, h-v.h};
    }

    Vec3D operator/ (double d)
    {
        return Vec3D{x/d, y/d, h/d};
    }
};

//struct Log: writes logs to a file with a special structure
struct Log {
private:
    std::ofstream file;
    std::ofstream centers_of_circles_file;
    std::ofstream touch_point_file;
    std::ofstream full_points_file; //здесь кллючевые точки маршрута + те, что добавлены при оптимизации
    int shift = 0;
public:
    Log() {}

    void init(std::string file_name){
        file.open(file_name);
        centers_of_circles_file.open("centers.txt");
        touch_point_file.open("touch_points.txt");
        full_points_file.open("points.txt");
    }

    void add_log(std::string text) {
        if (file) {
            for (int i = 0; i < shift; i++)
                file << "|\t";
            file << text << "\n";
        }
    }

    void start_process(std::string text) {
        if (file) {
            add_log(text);
            shift++;
        }
    }

    void add_center(double x, double y){
        centers_of_circles_file << x << " "<< y << "\n";
    }

    void add_touch_point(double x, double y){
        touch_point_file << x << " "<< y << "\n";
    }

    void add_point(double x, double y)
    {
        const auto digits = std::numeric_limits<double>::digits10;
        full_points_file << std::setfill(' ') << std::setw(digits + 4);
        full_points_file << std::fixed << std::setprecision(digits) << x << " " << y << "\n";
    }

    void finish_process(std::string text = "finished successfully") {
        if (file) {
            shift--;
            add_log(text);
        }
    }

    ~Log() {
        if (file)
            file.close();
        centers_of_circles_file.close();
        touch_point_file.close();
        full_points_file.close();
    }

};

struct ErrorPair {
    double dh;
    double dFi;
};


#endif //AVES_STRUCTS_H
