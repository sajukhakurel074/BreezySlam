#include "OurMap.hpp"
OurMap::OurMap( int size_pixels, int map_size_meters, int num_particles, bool init_pixels ):
  size_pixels(size_pixels), size_meters(map_size_meters), num_maps(num_particles),
  allocated_count(0){
    size_t npix = size_pixels * size_pixels  * num_maps ;
    maps = new pixel_t[  npix * sizeof(pixel_t)];
    if ( init_pixels ){
      for ( size_t i = 0; i < npix; i++ ){
        maps[i] = ( OBSTACLE + NO_OBSTACLE )/2;
      }
    }
    scale_pixels_per_mm = size_pixels / ( size_meters * 1000 );
  }

  void OurMap::update(
    int map_handle,
    Scan & scan, 
    Position & position, 
    int quality, 
    double hole_width_mm)
  {
    position_t cpos;
    cpos.x_mm = position.x_mm;
    cpos.y_mm = position.y_mm;
    cpos.theta_degrees = position.theta_degrees;
    
    map_t map;
    map.size_pixels = size_pixels;
    map.size_meters = size_meters;
    map.scale_pixels_per_mm = scale_pixels_per_mm;

    map.pixels = maps + size_pixels * size_pixels * map_handle;
    map_update( &map, scan.scan, cpos, quality, hole_width_mm );
  }

  pixel_t* OurMap::get_particle_map( int map_handle ){
    return maps + size_pixels * size_pixels * map_handle;
  }

  void OurMap::particle_set_map( int map_handle, pixel_t *map ){
    pixel_t *dest = maps + size_pixels * size_pixels * map_handle;
    memcpy( dest, map, size_pixels * size_pixels );
  }

  int OurMap::new_map(){
    if ( allocated_count >= num_maps ){
      std::cerr<< "Attempt to allocate more maps than provided!" << std::endl;
      exit(0);
    }
    int ret_val = allocated_count++;
    return ret_val;
  }

  int OurMap::add_new_map( pixel_t *src ){
    if ( allocated_count >= num_maps ){
      std::cerr<< "Attempt to allocate more maps than provided!" << std::endl;
      exit(0);
    }
    pixel_t *dest = maps + size_pixels * size_pixels * allocated_count;
    memcpy( dest, src, size_pixels * size_pixels * sizeof(pixel_t) );
    int ret_val = allocated_count++;
    return ret_val;
  }
    
OurMap::~OurMap( ){
    delete maps[];
  }

