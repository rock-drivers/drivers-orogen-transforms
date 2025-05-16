#ifndef transforms_TYPES_HPP
#define transforms_TYPES_HPP

#include <base/Angle.hpp>
#include <base/Float.hpp>
#include <base/Time.hpp>

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace transforms {
    struct PoseDivergence {
        base::Time time;
        bool position_divergent = false;
        bool roll_divergent = false;
        bool pitch_divergent = false;
        bool yaw_divergent = false;
        double position_error_norm = base::unknown<double>();
        base::Angle roll_error;
        base::Angle yaw_error;
        base::Angle pitch_error;
    };
}

#endif
