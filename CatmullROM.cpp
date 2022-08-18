#include "Lines.h"
#define SKIPPOINT else {points_full.pop_back(); points_full.pop_back(); points_full.push_back(points[i]);}

void CatmullROM::get_bis_coef_and_circ_center(std::vector<Vec3D>& points, int i, double min_R_in_d_lat, double& bis_k, double& bis_b, double& x0, double& y0)
{
    //есть текущая точка маршрута, а так же следующая и предыдущая
    //функция вычислет коэффициенты биссектрисы угла, образованного этими точками
    //и центр окружности радиуса min_R_in_d_lat, лежащий на этой биссектрисе
    double rho01 = points[i].lengh(points[i - 1]);
    double rho21 = points[i].lengh(points[i + 1]);

    Vec2D V10 = Vec2D{(points[i - 1].x - points[i].x), (points[i - 1].y - points[i].y)} / rho01;
    Vec2D V12 = Vec2D{(points[i + 1].x - points[i].x), (points[i + 1].y - points[i].y)} / rho21;

    Vec2D point12_ = V10*20 + points[i].get_2d();
    Vec2D point12 = V12*20 + points[i].get_2d();
    Vec2D mid_point = point12 + (point12_ - point12) / 2;

    //definition
    bis_k = (points[i].y - mid_point.y) / (points[i].x - mid_point.x);
    bis_b = points[i].y - bis_k*points[i].x;

    double len = sqrt(pow(mid_point.x - points[i].get_2d().x, 2) + pow(mid_point.y - points[i].get_2d().y, 2));
    Vec2D V1m = ((mid_point - points[i].get_2d()) / len) * (min_R_in_d_lat);

    Vec2D center = V1m + points[i].get_2d();

    //definition
    x0 = center.x;
    y0 = center.y;
}

void CatmullROM::get_touch_points(std::vector<Vec3D>& points, int j, double x0, double y0, double min_R_in_d_lat, Vec2D& touch_point1, Vec2D& touch_point2)
{
    double x1 = points[j].x, y1 = points[j].y;
    double k_1, k_2, b_1, b_2;

    double dx = x1-x0;
    double dy = y1-y0;

    if (x0 - x1 == min_R_in_d_lat) {
        touch_point1 = Vec2D{x0 - min_R_in_d_lat, y0};
        k_2 = -(pow(dy, 2) - pow(min_R_in_d_lat, 2)) / (2 * min_R_in_d_lat * dy);
        b_2 = y1 - k_2 * x1;
        double x = (2 * x0 - 2 * k_2 * b_2 + 2 * k_2 * y0) / (2 * (1 + pow(k_2, 2)));
        touch_point2 = Vec2D{x, k_2 * x + b_2};
    } else if (x0 - x1 == -min_R_in_d_lat) {
        touch_point1 = Vec2D{x0 + min_R_in_d_lat, y0};
        k_2 = (pow(dy, 2) - pow(min_R_in_d_lat, 2)) / (2 * min_R_in_d_lat * dy);
        b_2 = y1 - k_2 * x1;
        double x = (2 * x0 - 2 * k_2 * b_2 + 2 * k_2 * y0) / (2 * (1 + pow(k_2, 2)));
        touch_point2 = Vec2D{x, k_2 * x + b_2};
    } else {
        k_1 = (dx * dy + min_R_in_d_lat * sqrt(pow(dx, 2) + pow(dy, 2) - pow(min_R_in_d_lat, 2))) / (pow(dx, 2) - pow(min_R_in_d_lat, 2));
        k_2 = (dx * dy - min_R_in_d_lat * sqrt(pow(dx, 2) + pow(dy, 2) - pow(min_R_in_d_lat, 2))) / (pow(dx, 2) - pow(min_R_in_d_lat, 2));
        b_1 = y1 - k_1 * x1;
        b_2 = y1 - k_2 * x1;
        double x_1 = (2 * x0 - 2 * k_1 * b_1 + 2 * k_1 * y0) / (2 * (1 + pow(k_1, 2)));
        touch_point1 = Vec2D{x_1, k_1 * x_1 + b_1};

        double x_2 = (2 * x0 - 2 * k_2 * b_2 + 2 * k_2 * y0) / (2 * (1 + pow(k_2, 2)));
        touch_point2 = Vec2D{x_2, k_2 * x_2 + b_2};
    }
}

void CatmullROM::check_order(std::vector<Vec3D>& points_full)
{
    double d = points_full.size()-1;
    if ((points_full[d-1] - points_full[d-2]).get_2d().angel((points_full[d] - points_full[d-1]).get_2d()) > M_PI/2) {
        Vec3D tmp = points_full[d-1];
        points_full[d-1] = points_full[d];
        points_full[d] = tmp;
    }
}

void CatmullROM::add_first_touch_point(std::vector<Vec3D>& points_full, Vec2D touch_point1, Vec2D touch_point2, int i){
    if (points[i].get_2d().lengh(touch_point1) < points[i].get_2d().lengh(touch_point2))
        points_full.push_back(Vec3D{touch_point1, points[i].h});
    else
        points_full.push_back(Vec3D{touch_point2, points[i].h});
    points_full.push_back(points[i]);
    check_order(points_full);
}

void CatmullROM::add_second_touch_point(std::vector<Vec3D>& points_full, Vec2D touch_point1, Vec2D touch_point2, int i, double bis_k, double bis_b) {
    Vec3D prev_circ_point = points_full[points_full.size() - 2];
    double angle_touch1_i_old = (prev_circ_point.get_2d() - points[i].get_2d()).angel(touch_point1 - points[i].get_2d());
    double angle_touch2_i_old = (prev_circ_point.get_2d() - points[i].get_2d()).angel(touch_point2 - points[i].get_2d());
    double angle_next_i_old = (prev_circ_point.get_2d() - points[i].get_2d()).angel(points[i+1].get_2d() - points[i].get_2d());

    if (points[i].get_2d().lengh(prev_circ_point.get_2d()) > min_R/10) {
        if ((bis_k * prev_circ_point.x + bis_b - prev_circ_point.y) *
            (bis_k * touch_point1.x + bis_b - touch_point1.y) < 0) {
            if (angle_touch1_i_old > angle_next_i_old)
            {
                points_full.push_back(Vec3D{touch_point1, points[i].h});
                std::cout << i << " ifif\n";
            }
            SKIPPOINT
            std::cout << i << " skip1\n";
        } else {
            if (angle_touch2_i_old > angle_next_i_old)
            {
                points_full.push_back(Vec3D{touch_point2, points[i].h});
                std::cout << i << " ifelse\n";
            }
            SKIPPOINT
            std::cout << i << " skip2\n";
        }
    }
    else {
        if (points[i].get_2d().lengh(touch_point1) < points[i].get_2d().lengh(touch_point2)) {
            if (angle_touch1_i_old > angle_next_i_old)
            {
                points_full.push_back(Vec3D{touch_point1, points[i].h});
                std::cout << i << " elseif\n";
            }
            SKIPPOINT
            std::cout << i << " skip3\n";
        }
        else {
            if (angle_touch2_i_old > angle_next_i_old)
            {
                points_full.push_back(Vec3D{touch_point2, points[i].h});
                std::cout << i << " elseelse\n";
            }
            SKIPPOINT
            std::cout << i << " skip4\n";
        }
    }
}

void CatmullROM::build(){
    if (points.size() <= 2)
        return;

    double DEG_LON = 111003;
    double DEG_LAT = cos(points[0].x)*6360*2*M_PI*1000/360; // 1000 -- km to m
    double min_R_in_d_lat = ((min_R / DEG_LON) + (min_R / DEG_LAT))/2;

    std::vector<Vec3D> points_full;
    points_full.push_back(points[0]);

    for (int i = 1; i < points.size() - 1 ; i++) {
        double bis_k, bis_b, x0, y0;
        get_bis_coef_and_circ_center(points, i, min_R_in_d_lat, bis_k, bis_b, x0, y0);

        logs.add_center(x0, y0);

        for (int j = i - 1; j <= i + 1; j+=2) {
            Vec2D touch_point1, touch_point2;
            get_touch_points(points, j, x0, y0, min_R_in_d_lat, touch_point1, touch_point2);

            logs.add_touch_point(touch_point1.x, touch_point1.y);
            logs.add_touch_point(touch_point2.x, touch_point2.y);

            if (j == i-1)
                add_first_touch_point(points_full, touch_point1, touch_point2, i);
            else
                add_second_touch_point(points_full, touch_point1, touch_point2, i, bis_k, bis_b);
        }
    }
    points_full.push_back(points[points.size() - 1]);

    points = points_full;
    
    std::ofstream file;
    //file.open(files_addres);
    file.open("path_points.txt");
    
    for (auto &i: points) {
        const auto digits = std::numeric_limits<double>::digits10;
        file << std::setfill(' ') << std::setw(digits + 4);
        file << std::fixed << std::setprecision(digits) << std::to_string(i.x) << " " << std::to_string(i.y) << " " << std::to_string(i.h) << std::endl;
    }
    file.close();
};