#pragma once
#include "../ukf/types.h"


// field length
const double full = 144.0;
// inside offset
const double off = 0.25;
// field inner length
// const double fl = full - off * 2.0;
const double fl = full;



class Map {
public:
    std::vector<std::pair<V2d, V2d>> lines;

    Map() {
        std::vector<V2d> points;

        // leg
        points.emplace_back(V2d { 46.75, 22.25 });
        points.emplace_back(V2d { 49.85, 25.2 });
        points.emplace_back(V2d { 49.85, 25.2 });
        points.emplace_back(V2d { 46.75, 28.25 });

        // leg left top of the fielt
        points.emplace_back(V2d { 46.75, fl - 22.25 });
        points.emplace_back(V2d { 49.85, fl - 25.2 });
        points.emplace_back(V2d { 49.85, fl - 25.2 });
        points.emplace_back(V2d { 46.75, fl - 28.25 });

        // left y wall
        points.emplace_back(V2d { 0.0, 0.0 });
        points.emplace_back(V2d { 0.0, fl });       

        // bottom x wall
        points.emplace_back(V2d { 0.0, 0.0 });
        points.emplace_back(V2d { fl, 0.0 });

        // right y wall
        points.emplace_back(V2d { fl, 0.0 });
        points.emplace_back(V2d { fl, fl });

        // top x wall
        points.emplace_back(V2d { 0.0, fl });
        points.emplace_back(V2d { fl, fl });

        for(int i = 0; i < points.size(); i += 2) {
            lines.emplace_back(std::pair<V2d, V2d> { points.at(i), points.at(i+1) });
        }
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