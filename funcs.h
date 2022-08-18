#ifndef AVES_FUNCS_H
#define AVES_FUNCS_H

#include "structs.h"

#include <iomanip>
#include <limits>

std::string add_time(std::string);

ErrorPair calc_errors(Vec3D, Vec3D, Vec3D);


#endif //AVES_FUNCS_H
