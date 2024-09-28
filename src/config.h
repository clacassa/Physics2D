#ifndef CONFIG_H
#define CONFIG_H

#define FRICTION // 6/02/2024 WORKING!!
#define SWEEP_AND_PRUNE
#define SAT
#define GJK_EPA

#define IM_EULER

#ifdef DEBUG
#   ifdef FRICTION
#       define DEBUG_FRICTION
#   endif
#define DEBUG_COLLISION
#endif

constexpr unsigned min_substeps(1);
constexpr unsigned max_substeps(50);
constexpr double max_time_step(1.0 / 60.0);

constexpr double PI(3.14159265);
constexpr double g(9.81);
constexpr double air_viscosity(1.48e-5);

#endif /* CONFIG_H */
