#include "OurSlam.hpp"

Particle::Particle(double a,
    double b,
    double th, 
    double weight, 
    Map& map): x(a),y(b),theta(th),w(weight),m(map)
    {}

Particle::Particle():x(0),y(0),theta(0),w(1),m(800, 8){}

OurSlam::OurSlam(Laser & laser, 
    int map_size_pixels, 
    double map_size_meters):CoreSLAM(laser, map_size_pixels, map_size_meters)
    {
        void* random = random_new(7);
        for(int i=0;i<100;i++)
        {
            Map M(800, 8);
            const Particle P(random_normal(random, 4000, 400), random_normal(random, 4000, 400), random_normal(random, 45, 7), 1, M);
            particles.push_back(P);
        }
        for (int i = 0; i < 100; i++)
        {
            std::cout << particles[i].x << ' ';
            std::cout << particles[i].y << ' ' << endl;
            //std::cout << particles[i].theta << ' '<< endl;
        }    
    }


OurSlam::~OurSlam(){}

void OurSlam::updateMapAndPointcloud(PoseChange & poseChange)
{
    
}