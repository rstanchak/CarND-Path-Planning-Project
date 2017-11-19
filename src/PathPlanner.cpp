#include "PathPlanner.hpp"
#include "helpers.hpp"
#include "spline.h"

#define MS_IN_MPH 0.44704

// The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road.
// The lane is 4 m wide
#define LANE_WIDTH 4

double normalize_angle(double theta) {
    return atan2( sin(theta), cos(theta) );
}

struct MapToVehicleTransform {
    double cos_theta, sin_theta, tx, ty;
    MapToVehicleTransform(double px, double py, double theta):
        cos_theta(cos(theta)),
        sin_theta(sin(theta)),
        tx(-px),
        ty(-py) { }
	std::vector<double> operator()(double x, double y){
        return { 
			(x+tx)*cos_theta + (y+ty)*sin_theta, 
			-(x+tx)*sin_theta + (y+ty)*cos_theta 
		};
    }
};

struct VehicleToMapTransform {
    double cos_theta, sin_theta, tx, ty;
    VehicleToMapTransform(double px, double py, double theta):
        cos_theta(cos(-theta)),
        sin_theta(sin(-theta)),
        tx(px),
        ty(py) { }
	std::vector<double> operator()(double x, double y)
	{
		return {
			x*cos_theta + y*sin_theta + tx,
        	-x*sin_theta + y*cos_theta + ty};
    }
};

/** constructor */
PathPlanner::PathPlanner( Map map ) : 
	map_(map), 
	vref_(0) 
{
}

bool isBlocking( double self_s, double self_d, double other_s, double other_d )
{
	return (other_s > self_s && (other_s-self_s)<LANE_WIDTH*6 && fabs(self_d-other_d) < LANE_WIDTH*0.5 );
}

void PathPlanner::update(nlohmann::basic_json<>::value_type& j, std::vector<double> * retarg_x, std::vector<double> * retarg_y)
{

	// Main car's localization Data
	double car_x = j["x"];
	double car_y = j["y"];
	double car_s = j["s"];
	double car_d = j["d"];
	double car_yaw = deg2rad(j["yaw"]);
	double car_speed = ((double)j["speed"]) * MS_IN_MPH;

	// Previous path data given to the Planner
	auto previous_path_x = j["previous_path_x"];
	auto previous_path_y = j["previous_path_y"];
	// Previous path's end s and d values 
	double end_path_s = j["end_path_s"];
	double end_path_d = j["end_path_d"];

	// Sensor Fusion Data, a list of all other cars on the same side of the road.
	auto sensor_fusion = j["sensor_fusion"];

#if 0
	int waypoint_idx = NextWaypoint( car_x, car_y, car_yaw, map_.waypoints_x, map_.waypoints_y);
	retarg_x->push_back( map_.waypoints_x[ waypoint_idx ] );
	retarg_y->push_back( map_.waypoints_y[ waypoint_idx ] );
#endif
	//  The car moves 50 times a second or every 0.02s
	//  The target speed is 50 MPH which is 22.352 m/s 
	//  The target distance per move is thus 22.352/50 = 0.44704 m
	//
	// Acceleration is calculated by comparing the rate of change of average speed over .2 second intervals (or 10 hz)
	// The jerk is calculated as the average acceleration over 1 second intervals (or 1 hz)
	// The jerk and the total acceleration should not exceed 10 m/s^2.
	double vmax = 22.352 * 0.9; // 10% buffer
	double v = car_speed; 
	double a = 10 * 0.9; // 10% buffer
	double dt = 0.02;
	double lane_idx = 1;
	bool is_blocking=false;


	for(int i=0; i<sensor_fusion.size(); ++i)
	{
		// The data format for each car is: [ id, x, y, vx, vy, s, d]. 
		// The id is a unique identifier for that car. 
		// The x, y values are in global map coordinates,
		// the vx, vy values are the velocity components, also in reference to the global map. 
		// Finally s and d are the Frenet coordinates for that car.
		int id = sensor_fusion[i][0];
		double x = sensor_fusion[i][1];
		double y = sensor_fusion[i][2];
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double s = sensor_fusion[i][5];
		double d = sensor_fusion[i][6];
		if( isBlocking(car_s, car_d, s, d) )
		{
			is_blocking=true;
		}
	}
	double target_d = LANE_WIDTH * (0.5 + lane_idx);

	std::vector<double> pts_x;
	std::vector<double> pts_y;
	double vlast = vref_;
	if( is_blocking && vref_ > 0)
	{
		vref_ -= a*dt;
	}
	else if (vref_ < vmax)
	{
		vref_ += a*dt;
	}


	int prev_points_size = previous_path_x.size();
	double ref_x;
	double ref_y;
	double ref_yaw;

	for(int i=0; i<prev_points_size; ++i)
	{
		retarg_x->push_back(previous_path_x[i]);
		retarg_y->push_back(previous_path_y[i]);
	}
	
	if(prev_points_size<2)
	{
		ref_x = car_x;
		ref_y = car_y;
		ref_yaw = car_yaw;

	}
	else
	{
		ref_x = previous_path_x[prev_points_size-1];
		ref_y = previous_path_y[prev_points_size-1];
		double prev_x = previous_path_x[prev_points_size-2];
		double prev_y = previous_path_y[prev_points_size-2];
		ref_yaw = atan2( ref_y-prev_y, ref_x - prev_x );
	}

	MapToVehicleTransform map2car( ref_x, ref_y, ref_yaw );
	VehicleToMapTransform car2map( ref_x, ref_y, ref_yaw );

	// one point behind vehicle
	pts_x.push_back(ref_x - cos(ref_yaw));
	pts_y.push_back(ref_y - sin(ref_yaw));

	// one point at vehicle
	pts_x.push_back(ref_x);
	pts_y.push_back(ref_y);

	auto ref_sd = getFrenet( ref_x, ref_y, ref_yaw, map_.waypoints_x, map_.waypoints_y);
	double ref_s = ref_sd[0];

	// 90 meters ahead
	for(int i=0; i<3; ++i){
		double s = ref_s + (i+1)*30;
		auto mapxy = getXY(s, target_d, map_.waypoints_s, map_.waypoints_x, map_.waypoints_y);
		//std::cout<<car_x<<","<<car_y<<","<<car_yaw<<" s,d="<<s<<","<<d<<" map="<<mapxy[0]<<","<<mapxy[1]<<" car="<<carxy[0]<<","<<carxy[1]<<std::endl;

		pts_x.push_back(mapxy[0]);
		pts_y.push_back(mapxy[1]);
	}
	for(int i=0; i<pts_x.size(); ++i)
	{
		auto carxy = map2car(pts_x[i], pts_y[i]);
		pts_x[i] = carxy[0];
		pts_y[i] = carxy[1];
	}
	tk::spline spline;
	spline.set_points(pts_x, pts_y);

	for(int i = 0; i < (50-prev_points_size); i++)
	{
		double carx = (i+1)*vref_*dt;
		double cary = spline(carx);
		auto mapxy = car2map(carx, cary);

		retarg_x->push_back(mapxy[0]);
		retarg_y->push_back(mapxy[1]);
	}

}
