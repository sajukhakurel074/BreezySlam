#ifndef __OURSLAM_HPP__
#define __OURSLAM_HPP__

#include <algorithms.hpp>
#include <vector>
#include <Map.hpp>
#include <random.h>
#include "OurMap.hpp"


class Particle
{
    public:
        double x, y, theta, w;
        int map_handle;

    Particle(double x, double y, double theta, double w, int map_handle);
    Particle();
};

class OurSlam: public CoreSLAM
{
    std::vector<Particle> particles;
    OurMap map;

    protected:
        void updateMapAndPointcloud(PoseChange & poseChange);
    public:    
        OurSlam( Laser & laser, int map_size_pixels, double map_size_meters);
        ~OurSlam();
        

   
};


#endif