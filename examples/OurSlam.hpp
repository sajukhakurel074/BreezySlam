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
    void Print_Particles();
};

class OurSlam: public CoreSLAM
{
    std::vector<Particle> particles;
    OurMap map;
    void * randomizer;
    int number_of_particles;
    Particle most_important_point;

    protected:
        void updateMapAndPointcloud(PoseChange & poseChange);
    public:    
        OurSlam( Laser & laser, int map_size_pixels, double map_size_meters, int number_of_particles);
        inline const std::vector<Particle>& getParticles(){return particles;};
        ~OurSlam();
   
};


#endif