#include "PathPlanner.hpp"
#include "helpers.hpp"
#include "spline.h"

#define MS_IN_MPH 0.44704

// The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road.
// The lane is 4 m wide
#define LANE_WIDTH 4
#define MAX_LANES 3

// http://answers.google.com/answers/threadview/id/144173.html
// GM HUMMER - H1
// Overall length (in / mm) 
// without winch: 184.5 / 4686 
// with winch: 190.5 / 4839
//  
// Overall width (in / mm) 
// without mirrors: 86.5 / 2197 
// with mirrors: 101 / 2565
#define CAR_WIDTH_M 2.565
#define CAR_LENGTH_M 4.839

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
	vref_(-1),
	state_(STATE_KEEP_LANE),
	lane_ref_(-1) /* i.e. current lane */
{
}

#define OCC_NBINS 5
#define LANE(d) ((int)((d)/LANE_WIDTH))
int _calcBestLane( int current_lane, double sref, double vref, nlohmann::basic_json<>::value_type& sensor_fusion, double * vlanes, size_t nlanes)
{
	int occupancy[nlanes][OCC_NBINS];
	double vocc[nlanes][OCC_NBINS]; // minimum vehicle speed in this cell
	for(int i=0; i<nlanes; ++i)
	{
		for(int j=0; j<OCC_NBINS; ++j)
		{
			occupancy[i][j]=0;
			vocc[i][j]=1e10;
		}
	}

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
		double v = sqrt(vx*vx+vy*vy);

		// vehicle bounds in Frenet
		double s_bounds[2] = {s - CAR_LENGTH_M/2, s + CAR_LENGTH_M/2};
		double d_bounds[2] = {d -CAR_WIDTH_M/2, d + CAR_WIDTH_M/2};

		for(int j=0; j<4; ++j)
		{
			// mark occupancy grid
			double sj = s_bounds[j/2];
			double dj = d_bounds[j%2];
			int lanej = LANE(dj);
			int d = 0;

			// safety margin ~ 4 car lengths
			double m = (sj-sref)/(CAR_LENGTH_M*4);
			int mbin = floor(m + OCC_NBINS*0.5);
			// clamp bins
			if(mbin >= 0 && mbin < OCC_NBINS)
			{
				++occupancy[lanej][mbin];
				if(v<vocc[lanej][mbin])
				{
					vocc[lanej][mbin]=v;
				}
			}
		}
	}

#ifdef DEBUG
	std::cout<<"--------------------"<<std::endl;
	for(int i=0; i<nlanes; ++i)
	{
		for(int j=0; j<OCC_NBINS; ++j)
		{
			std::cout<<"\t | ("<<occupancy[i][j]<<","<<vocc[i][j]<<")";
		}
		std::cout <<"\t |"<<std::endl;
	}
	std::cout<<"--------------------"<<std::endl;
#endif

	int best_lane = current_lane;

	// for each lane,
	// compute minimum speed of occupancy bins in front of me
	for(int i=0; i<nlanes; ++i)
	{
		double minv = 1e10;
		for(int j=OCC_NBINS/2+1; j<OCC_NBINS; ++j)
		{
			if(vocc[i][j]<minv) minv=vocc[i][j];
		}
		vlanes[i] = minv;
	}

	// determine best lane based on reachability, occupancy and then minimum velocity and
	for(int i=0; i<nlanes; ++i)
	{
		if(abs(current_lane-i)>1) continue; // can't get there from here
		if(i!=current_lane && occupancy[i][OCC_NBINS/2]) continue; // the lane is occupied directly adjacent to my vehicle
		if(vlanes[i] > vlanes[best_lane]){
			best_lane = i;
		}
	}
	// TODO
	// 1. check occupancy bins behind me
	// 2. predict future location of vehicles
	return best_lane;
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

	int current_lane = car_d/LANE_WIDTH;

	// initialize state
	if(lane_ref_ < 0) lane_ref_ = current_lane;
	if(vref_ < 0) vref_ = car_speed;

	//  The car moves 50 times a second or every 0.02s
	//  The target speed is 50 MPH which is 22.352 m/s 
	//  The target distance per move is thus 22.352/50 = 0.44704 m
	//
	// Acceleration is calculated by comparing the rate of change of average speed over .2 second intervals (or 10 hz)
	// The jerk is calculated as the average acceleration over 1 second intervals (or 1 hz)
	// The jerk and the total acceleration should not exceed 10 m/s^2.
	double vmax = 22.352 * 0.99; // 1% buffer
	double v = car_speed; 
	double a = 10 * 0.9; // 10% buffer
	double dt = 0.02;

	std::vector<double> pts_x;
	std::vector<double> pts_y;

	double vlanes[MAX_LANES];
	// compute best_lane, and minimum velocity ahead for all lanes
	int best_lane = _calcBestLane( current_lane, car_s, car_speed, sensor_fusion, vlanes, MAX_LANES );

	// evaluate state
	switch(state_)
	{
		case STATE_KEEP_LANE:
			{
				// switch lanes if the best_lane is atleast 5 mph better than current lane
				if( best_lane != current_lane && (vlanes[best_lane]) > (vlanes[current_lane] + MS_IN_MPH*5.))
				{
					state_ = STATE_CHANGE_LANE;
					lane_ref_ = best_lane;
				}
			}
			break;
		case STATE_CHANGE_LANE:
			if(current_lane == lane_ref_)
			{
				state_ = STATE_KEEP_LANE;
			}
			break;
	}

	// accelerate/deccelerate to match velocity of current lane
	double v_current_lane = vlanes[current_lane];
	double vnext = vref_;
	if(vref_ > v_current_lane)
	{
		vnext = vref_ - a*dt;
		if(vnext < v_current_lane) vnext = v_current_lane;
	}
	else
	{
		vnext = vref_ + a*dt;
		if(vnext > v_current_lane) vnext = v_current_lane;
	}

	// clamp to maximum speed
	if (vnext > vmax) vnext = vmax;

	// reference speed has been computed
	vref_ = vnext;

	// update path points
	double ref_x;
	double ref_y;
	double ref_yaw;

	// initialize with unused previous path
	int prev_points_size = previous_path_x.size();
	for(int i=0; i<prev_points_size; ++i)
	{
		retarg_x->push_back(previous_path_x[i]);
		retarg_y->push_back(previous_path_y[i]);
	}
	
	if(prev_points_size<2)
	{
		// previous path is too small, use current position and yaw as seed for
		// new path points
		ref_x = car_x;
		ref_y = car_y;
		ref_yaw = car_yaw;

	}
	else
	{
		// use last two points in previous path as seed for
		// new path points
		ref_x = previous_path_x[prev_points_size-1];
		ref_y = previous_path_y[prev_points_size-1];
		double prev_x = previous_path_x[prev_points_size-2];
		double prev_y = previous_path_y[prev_points_size-2];
		ref_yaw = atan2( ref_y-prev_y, ref_x - prev_x );
	}

	// i.e. the center of the reference lane
	double target_d = LANE_WIDTH * (0.5 + lane_ref_);

	// coordinate transformation objects
	MapToVehicleTransform map2car( ref_x, ref_y, ref_yaw );
	VehicleToMapTransform car2map( ref_x, ref_y, ref_yaw );

	// To generate trajectory, fit spline through the following:

	// 1. one point behind vehicle
	pts_x.push_back(ref_x - cos(ref_yaw));
	pts_y.push_back(ref_y - sin(ref_yaw));

	// 2. one point at vehicle
	pts_x.push_back(ref_x);
	pts_y.push_back(ref_y);

	auto ref_sd = getFrenet( ref_x, ref_y, ref_yaw, map_.waypoints_x, map_.waypoints_y);
	double ref_s = ref_sd[0];

	// 3. three points ahead at 30, 60, 90 meters 
	for(int i=0; i<3; ++i){
		double s = ref_s + (i+1)*30;
		auto mapxy = getXY(s, target_d, map_.waypoints_s, map_.waypoints_x, map_.waypoints_y);
		//std::cout<<car_x<<","<<car_y<<","<<car_yaw<<" s,d="<<s<<","<<d<<" map="<<mapxy[0]<<","<<mapxy[1]<<" car="<<carxy[0]<<","<<carxy[1]<<std::endl;

		pts_x.push_back(mapxy[0]);
		pts_y.push_back(mapxy[1]);
	}
	// Transform from map to object space.  
	for(int i=0; i<pts_x.size(); ++i)
	{
		auto carxy = map2car(pts_x[i], pts_y[i]);
		pts_x[i] = carxy[0];
		pts_y[i] = carxy[1];
	}
	// compute spline
	tk::spline spline;
	spline.set_points(pts_x, pts_y);

	// generate up to 50 points for the trajectory
	for(int i = 0; i < (50-prev_points_size); i++)
	{
		double carx = (i+1)*vref_*dt;
		double cary = spline(carx);
		auto mapxy = car2map(carx, cary);

		retarg_x->push_back(mapxy[0]);
		retarg_y->push_back(mapxy[1]);
	}

}
