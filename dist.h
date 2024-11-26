#pragma once

struct Coordinates {
  double lat;
  double lon;

  Coordinates() : Coordinates(0, 0) {
  }

  Coordinates(double lat, double lon) : lat(lat), lon(lon) {
  }
};

// Returns the distance in miles between 2 points (lat1, long1) and
// (lat2, long2).  Latitudes are positive above the equator and
// negative below; longitudes are positive heading east of Greenwich
// and negative heading west.  Example: Chicago is (41.88, -87.63).
//
// NOTE: you may get slightly different results depending on which
// (lat, long) pair is passed as the first parameter.
double distBetween2Points(Coordinates p1, Coordinates p2);

// Returns the center Coordinate between (lat1, lon1) and (lat2, lon2)
// Reference: http://www.movable-type.co.uk/scripts/latlong.html
Coordinates centerBetween2Points(Coordinates p1, Coordinates p2);
