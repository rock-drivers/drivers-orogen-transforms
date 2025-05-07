#ifndef transforms_TYPES_HPP
#define transforms_TYPES_HPP

#include <base/Time.hpp>

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace transforms {
    struct PoseDivergence {
        bool different = false;
        base::Time time;
    };
}

#endif

