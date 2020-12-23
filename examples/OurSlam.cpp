#include "OurSlam.hpp"

Particle::Particle(double a,
    double b,
    double th, 
    double weight, 
    int m): x(a),y(b),theta(th),w(weight),map_handle(m)
    {}

Particle::Particle():x(0),y(0),theta(0),w(1),map_handle(-1){}

void Particle::Print_Particles()
{
        std::cout << this->x <<" "<<this->y <<" "<< this->theta <<" "<< this->w <<std::endl;
}

OurSlam::OurSlam(Laser & laser, 
    int map_size_pixels, 
    double map_size_meters, int number_of_particles):CoreSLAM(laser, map_size_pixels, map_size_meters),
    map(map_size_pixels, map_size_meters, 100, 1), randomizer(random_new(8)), number_of_particles(number_of_particles)
    {
        for(int i=0;i<number_of_particles;i++)
        {
            const Particle P(400, 400, 45, 1, map.new_map());
            particles.push_back(P);
        }
        std::cout << "initial" << std::endl;
        for (int i = 0; i < number_of_particles; i++)
        {
            particles[i].Print_Particles();
        }    
    }

OurSlam::~OurSlam(){}

void OurSlam::updateMapAndPointcloud(PoseChange & poseChange)
{
    double sum = 0;
    float most_important_particle = 0.0;
    void * randomizer = random_new(5);
    position_t position;
    for(int i=0;i<number_of_particles;i++)
    {
        position.x_mm = particles[i].x;
        position.y_mm = particles[i].y;
        position.theta_degrees = particles[i].theta;

        map_t Map = map.get_map_t(particles[i].map_handle); 
        position_t positions = rmhc_position_search(position, &Map, scan_for_distance->scan, 15, 2, 1000, randomizer);

        particles[i].x = positions.x_mm ;
        particles[i].y = positions.y_mm; 
        particles[i].theta = positions.theta_degrees; 

        Position pos(positions.x_mm, positions.y_mm, positions.theta_degrees);

        map.update(particles[i].map_handle, *scan_for_mapbuild, pos, this->map_quality, this->hole_width_mm);

        particles[i].w = distance_scan_to_map(&Map, scan_for_distance->scan, positions);
        sum += particles[i].w;
    }

    for (int i = 0; i < number_of_particles; i++)
    {
        particles[i].w = particles[i].w / sum;
    }
    for (int i = 0; i < number_of_particles; i++)
    {
        if(particles[i].w > most_important_particle)
        {
            most_important_particle = particles[i].w;
            most_important_point = Particle(particles[i].x, particles[i].y, particles[i].theta, particles[i].w, particles[i].map_handle);
        } 
    }
    std::cout << "updated positions" << std::endl;
    for (int i = 0; i < number_of_particles; i++)
        {
            particles[i].Print_Particles();
        }   
    std::cout << most_important_particle << std::endl;
    std::cout <<"Most Important Particle";
    most_important_point.Print_Particles();
}


    