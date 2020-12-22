#include <iostream>
#include <vector>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "coreslam.h"
#include "Position.hpp"
#include "algorithms.hpp"
#include "Map.hpp"
#include "Scan.hpp"

class OurMap {
  pixel_t *maps;
  int size_pixels;
  double size_meters;
  double scale_pixels_per_mm;
  int num_maps;
  int allocated_count;


  public:
    OurMap( int size_pixels, int map_size_meters, int num_particles, bool init_pixels );

  void update(
    int map_handle,
    Scan & scan, 
    Position & position, 
    int quality, 
    double hole_width_mm);

  pixel_t* get_particle_map( int map_handle );

  void particle_set_map( int map_handle, pixel_t *map );

  int new_map();

  int add_new_map( pixel_t *src );

  map_t get_map_t( int map_handle );
    
~OurMap( );
};

