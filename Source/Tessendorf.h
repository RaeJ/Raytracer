#ifndef TESSENDORF_H
#define TESSENDORF_H

#define PI 3.14159265

#include <glm/glm.hpp>
#include "TestModelH.h"
#include <complex>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

typedef struct
{
  float height;
  int side_points;
  std::vector<vec4> geometric_points;
} Grid;

float ACTUAL_WIDTH = 2;
float HALF_W = ACTUAL_WIDTH/2;
vec4 SHIFT( HALF_W, HALF_W, HALF_W, HALF_W );

Grid GRID;
vector<Triangle> waves;

// Water Stuff
float gravity = 9.8;
glm::vec2 wind_dir( 0.424, 0.316 );
float max_wave = 0.4;
float amplitude = 0.02;
// float max_wave = ( glm::dot( wind_dir, wind_dir ) ) / gravity;
float step = HALF_W/5;
// int N = 64;
// int M = 64;

double e_r = 0.5;
double e_i = 0.5;

void CreateSurface( int triangle_number, float height, double time );
void UpdateHeight( double time );
void AddTessendorfWaves();

void CreateSurface( int triangle_number, float height, double time  )
{
  waves.clear();
  waves.reserve( pow( triangle_number, 2 ) * 2 );

  int width_points = triangle_number + 1;
  int total_points = pow( width_points, 2 );
  float triangle_size = ACTUAL_WIDTH / (float) triangle_number;

  std::vector<vec4> grid_points( total_points );
  for( int i=0; i<total_points; i++ ){
    grid_points[i].x = ( i % width_points ) * triangle_size;
    grid_points[i].z = height;
    grid_points[i].y = floor( i / width_points ) * triangle_size;

    grid_points[i]    -= SHIFT;
    grid_points[i].x  *= 1;
    grid_points[i].y  *= 1;
    grid_points[i].w   = 1.0;
  }
  Grid new_grid;
  new_grid.geometric_points = grid_points;
  new_grid.side_points = width_points;
  new_grid.height = height;

  GRID = new_grid;

  UpdateHeight( time );
}

void UpdateHeight( double time )
{
  // if( fmod( time, 2.0 ) == 0 ){
  //   e_r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  // }

  float multiplier = ( 1 / sqrt( 2 ) ) ;

  for( int i = 0; i<GRID.geometric_points.size(); i++ ){
    glm::vec2 x_0( GRID.geometric_points[i].x, GRID.geometric_points[i].y );
    std::complex<float> result = 0;

    for( float n = 0; n < HALF_W; n = n + step ){
      for( float m = 0; m < HALF_W; m = m + step ){
        float k_x = ( 2 * PI * ( n - HALF_W/2 ) ) / max_wave;
        float k_z = ( 2 * PI * ( m - HALF_W/2 ) ) / max_wave;

        glm::vec2 k( k_x, k_z );
        float nk = glm::length( k );
        float w = sqrt( gravity * nk );

        float k4 = pow( nk, 4 );

        std::complex<float> eikx0( cos( glm::dot( k, x_0 ) ), sin( glm::dot( k, x_0 ) ) );

        std::complex<float> neg_exp( cos( -time * w ), sin( -time * w ) );
        std::complex<float> pos_exp( cos( time * w ), sin( time * w ) );

        float phil_pos = 0; float phil_neg = 0;

        if( k4 != 0 ){
          phil_pos = amplitude * ( exp( -1 / ( glm::dot( k, k ) ) ) / k4 )
                               * pow( abs( glm::dot( k, wind_dir ) ), 2 );
          phil_neg = amplitude * ( exp( -1 / ( glm::dot( k, k ) ) ) / k4 )
                               * pow( abs( glm::dot( -k, wind_dir ) ), 2 );
        }


        std::complex<float> sqr_p( sqrt( phil_pos ) );
        std::complex<float> sqr_n( sqrt( phil_neg ) );

        std::complex<float> random_vars( e_r, e_i );

        std::complex<float> part_a = multiplier * random_vars * sqr_p;
        std::complex<float> part_b = multiplier * random_vars * sqr_n;
        std::complex<float> partial = ( ( part_a * pos_exp ) + ( part_b * neg_exp ) );

        result +=  partial * eikx0;
      }
    }
    GRID.geometric_points[i].z = abs( result );
    GRID.geometric_points[i].z += GRID.height;
    // GRID.geometric_points[i].y *= -1;
  }
  AddTessendorfWaves();
}

void AddTessendorfWaves(){
  int width = GRID.side_points;
  for( int col=0; col<width; col++ ){
    for( int row=0; row<width; row++ ){
      int i = ( row * width ) + col;
      if( ( i % width ) != ( width - 1 ) && ( row % width ) != ( width - 1 ) ){
        vec4 l      = GRID.geometric_points[i];
        vec4 r      = GRID.geometric_points[i + 1];
        vec4 down_l = GRID.geometric_points[i + width];
        vec4 down_r = GRID.geometric_points[i + width + 1];

        Triangle A = Triangle( down_r, down_l, r, vec3(0.25,0.34,0.96), true, false );
        waves.push_back( A );

        Triangle B = Triangle( down_l, l, r, vec3(0.25,0.34,0.96), true, false );
        waves.push_back( B );
      }
    }
  }
}

#endif
