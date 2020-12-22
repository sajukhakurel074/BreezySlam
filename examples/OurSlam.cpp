#include "OurSlam.hpp"

Particle::Particle(double a,
    double b,
    double th, 
    double weight, 
    int m): x(a),y(b),theta(th),w(weight),map_handle(m)
    {}

Particle::Particle():x(0),y(0),theta(0),w(1),map_handle(-1){}


OurSlam::OurSlam(Laser & laser, 
    int map_size_pixels, 
    double map_size_meters):CoreSLAM(laser, map_size_pixels, map_size_meters),
    map(map_size_pixels, map_size_meters, 100, 1)
    {
        for(int i=0;i<100;i++)
        {
            const Particle P(400, 400, 45, 1, map.new_map());
            particles.push_back(P);
        }
        for (int i = 0; i < 100; i++)
        {
            std::cout << particles[i].x << ' ';
            std::cout << particles[i].y << ' ';
            std::cout << particles[i].theta << ' '<< endl;
        }    
    }


OurSlam::~OurSlam(){}

void OurSlam::updateMapAndPointcloud(PoseChange & poseChange)
{
    void * randomizer = random_new(5);
    position_t position;
    
    for(int i=0;i<100;i++)
    {
        position.x_mm = particles[i].x;
        position.y_mm = particles[i].y;
        position.theta_degrees = particles[i].theta;

        position_t positions = rmhc_position_search(position, map_handler, scan, 15, 2, 1000, randomizer);
    }

}