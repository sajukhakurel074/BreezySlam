#include "OurSlam.hpp"
#define PRINT 0

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
    map(new OurMap(map_size_pixels, map_size_meters, number_of_particles, 1)), randomizer(random_new(8)), number_of_particles(number_of_particles), 
    map_size_pixels(map_size_pixels), map_size_meters(map_size_meters), Sampling_count(0)
    {
        for(int i=0;i<number_of_particles;i++)
        {
            const Particle P(4000, 4000, 45, 1, map->new_map());
            particles.push_back(P);
        }
        #if PRINT
            std::cout << "initial" << std::endl;
            for (int i = 0; i < number_of_particles; i++)
            {
                particles[i].Print_Particles();
            }  
        #endif      
    }

OurSlam::~OurSlam(){}

void OurSlam::updateMapAndPointcloud(PoseChange & poseChange)
{
    double sum = 0;
    float most_important_particle = 0.0;
    void * randomizer = random_new(5);
    position_t position;
    for(int i=0;i<particles.size();i++)
    {
        position.x_mm = particles[i].x;
        position.y_mm = particles[i].y;
        position.theta_degrees = particles[i].theta;

        map_t Map = map->get_map_t(particles[i].map_handle); 
        position_t positions = rmhc_position_search(position, &Map, scan_for_distance->scan, 15, 45, 1000, randomizer);

        particles[i].x = positions.x_mm ;
        particles[i].y = positions.y_mm; 
        particles[i].theta = positions.theta_degrees; 

        Position pos(positions.x_mm, positions.y_mm, positions.theta_degrees);

        map->update(particles[i].map_handle, *scan_for_mapbuild, pos, this->map_quality, this->hole_width_mm);

        particles[i].w = distance_scan_to_map(&Map, scan_for_distance->scan, positions) * particles[i].w;
        sum += particles[i].w;
    }
    #if PRINT
        std::cout << "updated positions" << std::endl;
        for (int i = 0; i < number_of_particles; i++)
            {
                particles[i].Print_Particles();
            }   
        std::cout << most_important_particle << std::endl;
        std::cout <<"Most Important Particle ";
        most_important_point.Print_Particles();
    #endif
    for (int i = 0; i < particles.size(); i++)
    {
        particles[i].w = particles[i].w / sum;
    }

    if(Sampling_count % 10 == 0)
    {
        particles = Resampling();
        Sampling_count = 0;
    }

    Sampling_count++;

    for (int i = 0; i < particles.size(); i++)
    {
        if(particles[i].w > most_important_particle)
        {
            most_important_particle = particles[i].w;
            most_important_point = Particle(particles[i].x, particles[i].y, particles[i].theta, particles[i].w, particles[i].map_handle);
        } 
    }
}

std::vector<Particle> OurSlam:: Resampling()
    {
        OurMap * New_MAP = new OurMap(map_size_pixels, map_size_meters, number_of_particles, 1);
        std::vector<Particle> Resampled_Particles;
        int i = 0;
        float r = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/((1.0/particles.size())-0)));        
        float c = particles[i].w;
        for(int m=1;m<particles.size()+1;m++)
        {
            float U = r + (m - 1)* (1.0/particles.size());
            while(U > c)
            {
                i = (i+1) % particles.size();
                c = c + particles[i].w; 
            }
            Resampled_Particles.push_back(particles[i]);
        }
        
        for(int j = 0; j < Resampled_Particles.size() ; j++)
        {
            pixel_t *s = map->get_particle_map(Resampled_Particles[j].map_handle);
            Resampled_Particles[j].map_handle = New_MAP->add_new_map(s);
            Resampled_Particles[j].w = 1;
        }
        delete map;
        map = New_MAP;
        return Resampled_Particles;
    }

pixel_t* OurSlam::getBestMap()
{
    return map->get_particle_map(most_important_point.map_handle);   
}




    