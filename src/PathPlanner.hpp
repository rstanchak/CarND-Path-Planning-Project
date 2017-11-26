#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include "Map.hpp"
#include "json.hpp"

class PathPlanner {
	Map map_;
	double vref_;
	enum State {
		STATE_KEEP_LANE,
		STATE_CHANGE_LANE
	} state_;
	int lane_ref_;
public:
	PathPlanner( Map map );
	void update(nlohmann::basic_json<>::value_type&, std::vector<double> * retarg_next_x_vals, std::vector<double> * retarg_next_y_vals);
};
#endif // PATH_PLANNER_HPP
