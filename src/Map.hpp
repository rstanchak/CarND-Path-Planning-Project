#ifndef MAP_HPP
#define MAP_HPP

#include <vector>

struct Map {
	// waypoint's x,y,s and d normalized normal vectors
	std::vector<double> waypoints_x;
	std::vector<double> waypoints_y;
	std::vector<double> waypoints_s;
	std::vector<double> waypoints_dx;
	std::vector<double> waypoints_dy;
};

#endif
