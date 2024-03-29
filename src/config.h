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

constexpr double PI(3.14159265);
constexpr double g(9.81);

#endif /* CONFIG_H */
