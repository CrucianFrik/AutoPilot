#ifndef FILTERS_H
#define FILTERS_H

struct ExpRunAverFilter{
private:
  double filVal = 0.0;
  double k = 0.1; //коэффициент сглаживания (0.0 – 1.0), чем он меньше, тем плавнее фильтр
public:
  ExpRunAverFilter(double kk, double fv = 0.0) {filVal = fv; k = kk;}
  double get_filtered(double newVal){
    filVal += (newVal - filVal) * k;
    return  filVal;
  }
};

#endif