#ifndef __OURSLAM_HPP__
#define __OURSLAM_HPP__

#include <algorithms.hpp>
#include <vector>
#include <Map.hpp>
#include <random.h>

class Particle
{
    public:
        double x, y, theta, w;
        Map m;

    Particle(double x, double y, double theta, double w, Map& m);
    Particle();
};

class OurSlam: public CoreSLAM
{
    std::vector<Particle> particles;
    protected:
        void updateMapAndPointcloud(PoseChange & poseChange);
    public:    
    OurSlam( Laser & laser, int map_size_pixels, double map_size_meters);
    ~OurSlam();
};

#endif