#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace oxebots_comms
{

inline constexpr double kDefaultMaxKickSpeedMps = 3.0;

inline double sanitize_max_kick_speed(double max_kick_speed_mps)
{
    return (std::isfinite(max_kick_speed_mps) && max_kick_speed_mps > 0.0)
               ? max_kick_speed_mps
               : kDefaultMaxKickSpeedMps;
}

// Converte kick_speed em m/s [0, max_kick_speed_mps] para escala 0..100 do SingleRobotVelCmd (uint7).
// Valores finitos > 0 sempre mapeiam para ao menos 1 para não descartar chutes fracos por arredondamento.
inline uint8_t kick_speed_to_team_percent(
    double kick_speed_mps,
    double max_kick_speed_mps = kDefaultMaxKickSpeedMps)
{
    if (!std::isfinite(kick_speed_mps) || kick_speed_mps <= 0.0) {
        return 0;
    }
    const double max_mps = sanitize_max_kick_speed(max_kick_speed_mps);
    const double normalized = std::clamp(kick_speed_mps / max_mps, 0.0, 1.0);
    const auto scaled = static_cast<int>(std::llround(normalized * 100.0));
    return static_cast<uint8_t>(std::clamp(scaled, 1, 100));
}

// Converte kick_speed em m/s [0, max_kick_speed_mps] para escala 0..255 do RobotCommand (uint8).
inline uint8_t kick_speed_to_u8(
    double kick_speed_mps,
    double max_kick_speed_mps = kDefaultMaxKickSpeedMps)
{
    if (!std::isfinite(kick_speed_mps) || kick_speed_mps <= 0.0) {
        return 0;
    }
    const double max_mps = sanitize_max_kick_speed(max_kick_speed_mps);
    const double normalized = std::clamp(kick_speed_mps / max_mps, 0.0, 1.0);
    const auto scaled = static_cast<int>(std::llround(normalized * 255.0));
    return static_cast<uint8_t>(std::clamp(scaled, 1, 255));
}

}  // namespace oxebots_comms
