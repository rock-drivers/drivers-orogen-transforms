#ifndef transforms_TYPES_HPP
#define transforms_TYPES_HPP

#include <base/Angle.hpp>
#include <base/Float.hpp>
#include <base/Time.hpp>
#include <gps_base/BaseTypes.hpp>

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace transforms {
    struct GPSInfo {
        base::Time time;
        gps_base::GPS_SOLUTION_TYPES main_gps_solution;

        static GPSInfo fromSolution(gps_base::Solution const& s)
        {
            return {s.time, s.positionType};
        }
    };
    struct PoseDivergence {
        base::Time time;
        bool position_divergent = false;
        bool roll_divergent = false;
        bool pitch_divergent = false;
        bool yaw_divergent = false;
        float position_error = base::unknown<float>();
        base::Angle roll_error;
        base::Angle yaw_error;
        base::Angle pitch_error;
        GPSInfo main_gps;
    };
    struct AngleErrorThresholds {
        base::Angle roll;
        base::Angle pitch;
        base::Angle yaw;
    };
}

#endif
