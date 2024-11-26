#include "dist.h"

#include <cmath>

double distBetween2Points(Coordinates p1, Coordinates p2) {
  // Reference: http://www8.nau.edu/cvm/latlon_formula.html
  double PI = 3.14159265;
  double earth_rad = 3963.1;  // statue miles:

  double lat1_rad = p1.lat * PI / 180.0;
  double long1_rad = p1.lon * PI / 180.0;
  double lat2_rad = p2.lat * PI / 180.0;
  double long2_rad = p2.lon * PI / 180.0;

  double dist =
      earth_rad *
      acos((cos(lat1_rad) * cos(long1_rad) * cos(lat2_rad) * cos(long2_rad)) +
           (cos(lat1_rad) * sin(long1_rad) * cos(lat2_rad) * sin(long2_rad)) +
           (sin(lat1_rad) * sin(lat2_rad)));

  return dist;
}

Coordinates centerBetween2Points(Coordinates p1, Coordinates p2) {
  double PI = 3.14159265;

  // convert to radians
  double lat1_rad = p1.lat * PI / 180.0;
  double long1_rad = p1.lon * PI / 180.0;
  double lat2_rad = p2.lat * PI / 180.0;
  double long2_rad = p2.lon * PI / 180.0;

  double long_diff = long2_rad - long1_rad;
  double Bx = cos(lat2_rad) * cos(long_diff);
  double By = cos(lat2_rad) * sin(long_diff);

  double lat_ret =
      atan2(sin(lat1_rad) + sin(lat2_rad),
            sqrt((cos(lat1_rad) + Bx) * (cos(lat1_rad) + Bx) + By * By));
  double long_ret = long1_rad + atan2(By, cos(lat1_rad) + Bx);

  // convert to degrees
  lat_ret = lat_ret * 180.0 / PI;
  long_ret = long_ret * 180.0 / PI;

  return Coordinates(lat_ret, long_ret);
}
