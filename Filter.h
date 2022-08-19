#include <algorithm>
#include <iostream>

struct ExpRunAverFilter{
private:
  double filVal = 0.0;
  double k = 0.1; //коэффициент сглаживания (0.0 – 1.0), чем он меньше, тем плавнее фильтр
  double fillDif = 0.0;
  double kDif = 0.2;
public:
  ExpRunAverFilter(double kk, double kkDif=0.2, double fv=0) 
  {k = kk; kDif = kkDif; filVal = fv;}
  
  double get_filtered(double newVal){
    if (filVal == 0)
    	filVal = newVal;
    double dif = (newVal - filVal);
    fillDif += (dif - fillDif) * k;
    filVal += dif * k;
    return  filVal;
  }
};

struct MedianFilter{
private:
  double val [3] {0, 0, 0};
  int flag = 0;
  int i = -1;
public:
  MedianFilter() {}
  double get_filtered(double newVal){
	i = (i+1)%3;
  	if (flag == 0) { flag++; val[0] = newVal; return newVal;}
  	else if (flag == 1) { flag++; val[1] = newVal; return newVal;}
  	else
  	{
  		val[i] = newVal;
  		float middle; 
  		if (std::max(val[0], val[1]) == std::max(val[1], val[2]))
	   		middle = std::max(val[0], val[2]);
  		else 
	  		middle = std::max(val[1], std::min(val[0], val[2]));
  		return middle;
  	}
  }
};

struct Filter{
private:
  ExpRunAverFilter ERAF {0, 0};
  MedianFilter MF;
  double prevVal=0;
public:
  Filter(double kk, double kkDif=0.2, double fv=0) { ERAF = ExpRunAverFilter(kk, kkDif, fv);}
  double get_filtered(double newVal){
    if (newVal==newVal)
    {
	    double tmp = MF.get_filtered(newVal);
	    tmp = ERAF.get_filtered(tmp);
	    prevVal = tmp;
	    return tmp;
    }
    else { return prevVal; }
  }
};



