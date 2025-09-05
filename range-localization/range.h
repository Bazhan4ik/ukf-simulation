#pragma once
#include "../ukf/types.h"

class Map {
public:
    std::vector<std::pair<V2d, V2d>> lines;

    Map() {
        V2d lg1p1 { 46.75, 22.25 };
        V2d lg1p2 { 49.85, 25.2 };

        V2d lg2p1 { 49.85, 25.2 };
        V2d lg2p2 { 46.75, 28.25 };
        
        V2d wall1p1 { 0.0, 0.0 };
        V2d wall1p2 { 0.0, 144.0 };

        V2d wall2p1 { 0.0, 0.0 };
        V2d wall2p2 { 144.0, 0.0 };

        V2d wall3p1 { 144.0, 0.0 };
        V2d wall3p2 { 144.0, 144.0 };

        auto wallpair = std::pair<V2d, V2d> { wall1p1, wall1p2 };
        auto wall2pair = std::pair<V2d, V2d> { wall2p1, wall2p2 };
        auto wall3pair = std::pair<V2d, V2d> { wall3p1, wall3p2 };

        auto pair1 = std::pair<V2d, V2d> { lg1p1, lg1p2 };
        auto pair2 = std::pair<V2d, V2d> { lg2p1, lg2p2 };



        lines.emplace_back(pair1);
        lines.emplace_back(pair2);
        lines.emplace_back(wallpair);
        lines.emplace_back(wall2pair);
        lines.emplace_back(wall3pair);
    }

    std::pair<double, V2d> detectDistance(V2d position, double theta) {
        V2d sd { cos(theta), sin(theta) };

        double dist = 150.0;
        V2d cp { 150.0, 150.0 };

        for(const auto &line : lines) {
            const V2d v1 = line.first;
            const V2d v2 = line.second;

            V2d dr = v2 - v1;
            double d = sd(0) * dr(1) - sd(1) * dr(0);

            if(fabs(d) < 1e-8) {
                continue;
            }

            V2d df = v1 - position;
            double t = (df(0) * dr(1) - df(1) * dr(0)) / d;
            double s = (df(0) * sd(1) - df(1) * sd(0)) / d;

            if(t >= 0 && s >= 0 && s <= 1 && t < dist) {
                dist = t;
                cp = t * sd;
                continue;
            }
        }

        return std::pair { dist, cp };
    }

};