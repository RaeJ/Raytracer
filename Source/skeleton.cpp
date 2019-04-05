#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "CastPhotons.h"

/* ----------------------------------------------------------------------------*/
/* STRUCTS                                                                     */


/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */

vector<Triangle> triangles;

vec3 theta( 0.0, 0.0, 0.0 );
vec3 indirect_light = 0.5f*vec3( 1, 1, 1 );

AABB root_aabb;
Node* root;
mat4 root_matrix;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update();
void Draw( screen* screen,
           vector<PhotonBeam> beams,
           vector<PhotonSeg>& items );

void TransformationMatrix( glm::mat4& m );
bool UserInput();
vec3 DirectLight( const Intersection& i );
void BeamRadiance( screen* screen,
                   vec4 start,
                   vec4 dir,
                   const Intersection& limit,
                   Node* parent,
                   vec3& current,
                   vector<PhotonBeam>& beams
                 );
float Integral_721( PhotonSeg s,
                    CylIntersection i,
                    float extinction,
                    vec4 dir  );
float Integral_722_ada( PhotonSeg s,
                        CylIntersection i,
                        float extinction,
                        vec4 dir );
float Integral_73( PhotonSeg s,
                   CylIntersection i,
                   float extinction,
                   vec4 dir  );

void Testing( screen* screen,
              vec4 start,
              vec4 direction,
              vec3& colour,
              vec3& current,
              vector<PhotonBeam> beams,
              vector<PhotonSeg>& items,
              const mat4& matrix );
void RecurseTree( Node* parent, const mat4& transform );

// ------------------------------------------------------------------------- //

void Testing( screen* screen,
  vec4 start,
  vec4 direction,
  vec3& colour,
  vec3& current,
  vector<PhotonBeam> beams,
  vector<PhotonSeg>& items,
  const mat4& matrix ){
    Intersection c_i;
    PhotonSeg debugging; CylIntersection intersect;
    debugging.orig_start = matrix * vec4( 0.0, -0.4, -0.2, 1.0f );
    debugging.start = matrix * vec4( -0.3, -0.2, -0.5, 1.0f );
    debugging.end = matrix * vec4( -0.9, 0.2, -1.1, 1.0f );
    debugging.radius = 0.05;
    PositionShader( screen, debugging.start, vec3( 1, 1, 0 ) );
    PositionShader( screen, debugging.end, vec3( 1, 1, 0 ) );
    if( HitCone( start, direction, debugging, intersect ) ){
      colour = vec3( 0.3, 0, 0.7 );
      vec4 light_location = matrix * light_position;
      vec4 position       = vec4( intersect.entry_point, 1.0f );
      vec3 radius = vec3 ( light_location ) - vec3( position );
      float A = 4 * PI * glm::dot( radius, radius );
      vec4 normal = vec4( intersect.entry_normal, 1.0f );
      float r_dot_n = glm::dot( glm::normalize( radius ), glm::normalize( vec3( normal ) ) );
      r_dot_n = max( r_dot_n, 0.0f );
      vec4 direction = glm::normalize( vec4( radius, 1.0f ) );
      vec4 start = position + 0.001f * normal;
      Intersection c_i;
      ClosestIntersection( start, direction, c_i, matrix, triangles );
      if( c_i.distance < glm::length( start - light_location ) ){
        r_dot_n = 0;
      }
      current += ( light_power * r_dot_n ) / A;
    } else if( ClosestIntersection( start, direction, c_i, matrix, triangles )  ){
      Triangle close = triangles[c_i.index];
      if( c_i.water ){
        close = waves[ c_i.index - triangles.size() ];
      }

      colour = close.colour;
      current += DirectLight( c_i );
    // PhotonBeam b;
    // b.start = debugging.start;
    // b.end = debugging.end;
    // DrawBeam( screen, b, vec3(0,1,0) );
  }
}

int main( int argc, char* argv[] )
{
  srand (time(NULL));
  CreateSurface( 10, 0.0, 1.004 );

  vector<PhotonBeam> beams;
  vector<PhotonSeg> items;

  LoadTestModel( triangles );

  cout << "Casting photons" << endl;
  mat4 matrix;  TransformationMatrix( matrix );
  root_matrix = matrix;
  root_aabb = CastPhotonBeams( PHOTON_NUMBER, beams, matrix, triangles );
  BoundPhotonBeams( beams, items, triangles );
  cout << "Beams size: " << beams.size() << "\n";
  cout << "Segment size: " << items.size() << "\n";
  root = newNode( root_aabb );
  cout << "Building tree" << endl;
  BuildTree( root, items );
  cout << "Calculating radiance" << endl;

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  // Draw( screen, beams, items );

  while( Update() )
    {
      Draw( screen, beams, items );
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

void RecurseTree( Node* parent, const mat4& transform ){
  Node* left     = parent->left;
  Node* right    = parent->right;
  if( left != NULL){
    left->aabb.min = transform * left->aabb.min;
    left->aabb.max = transform * left->aabb.max;
    left->aabb.mid = transform * left->aabb.mid;
    for( int i = 0; i<2; i++ ){
      if( left->segments[i].id != -1 ){
        left->segments[i].start = transform * left->segments[i].start;
        left->segments[i].end   = transform * left->segments[i].end;
        left->segments[i].mid   = transform * left->segments[i].mid;
        left->segments[i].max   = transform * left->segments[i].max;
        left->segments[i].orig_start = transform * left->segments[i].orig_start;
      }
    }
    RecurseTree( left, transform );
  }
  if( right != NULL){
    right->aabb.min = transform * right->aabb.min;
    right->aabb.max = transform * right->aabb.max;
    right->aabb.mid = transform * right->aabb.mid;
    for( int i = 0; i<2; i++ ){
      if( right->segments[i].id != -1 ){
        right->segments[i].start = transform * right->segments[i].start;
        right->segments[i].end   = transform * right->segments[i].end;
        right->segments[i].mid   = transform * right->segments[i].mid;
        right->segments[i].max   = transform * right->segments[i].max;
        right->segments[i].orig_start = transform * right->segments[i].orig_start;
      }
    }
    RecurseTree( right, transform );
  }
}

void Draw( screen* screen, vector<PhotonBeam> beams, vector<PhotonSeg>& items )
{
  mat4 inverse_matrix = glm::inverse( root_matrix );
  mat4 matrix;

  TransformationMatrix( matrix );

  // if( updated ){
  //   mat4 full_transform = matrix * inverse_matrix;
  //   full_transform[0][3] = 0; full_transform[2][3] = 0;
  //   full_transform[1][3] = 0; full_transform[3][3] = 1;
  //   RecurseTree( root, full_transform );
  //   root_matrix = matrix;
  // }
  /* Clear buffer */
  // memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  // Drawing stage
  for( int x = 0; x < (SCREEN_WIDTH * SSAA); x+=SSAA ) {
    for( int y = 0; y < (SCREEN_HEIGHT * SSAA); y+=SSAA ) {
      vec3 current = vec3( 0, 0, 0 );
      vec3 colour = vec3( 0, 0, 0 );
      for( int i = 0; i<SSAA; i++ ){
        for( int j = 0; j<SSAA; j++ ){
          float x_dir = ( x + i ) - ( (SCREEN_WIDTH * SSAA) / (float) 2 );
          float y_dir = ( y + j ) - ( (SCREEN_HEIGHT * SSAA) / (float) 2);

          vec4 direction = vec4( x_dir, y_dir, focal, 1.0);
          vec4 start = vec4( 0, 0, 0, 1 );
          Intersection c_i;

          direction = glm::normalize( direction );
          if( ClosestIntersection( start, direction, c_i, matrix, triangles ) ){
            Triangle close = triangles[c_i.index];
            colour = close.colour;

            BeamRadiance( screen, start, direction, c_i, root, current, beams );
          }
          // Testing( screen, start, direction, colour, current, beams, items, matrix );
        }
      }
      // PutPixelSDL( screen, x / SSAA, y / SSAA, colour / (float) SSAA );
      PutPixelSDL( screen, x / SSAA, y / SSAA, current / (float) SSAA );
    }

    // for( int i=0; i<segments.size(); i++ ){
    //   PositionShader( screen, segments[i].start, vec3(1,0,1));
    //   PositionShader( screen, segments[i].end, vec3(0,1,1));
    //   // AABB box; box.min = segments[i].min; box.max = segments[i].max;
    //   // DrawBoundingBox( screen, box );
    // }
    SDL_Renderframe(screen);
  }
}

bool Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  // std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
  return( UserInput() );
}

float Integral_721_ada( PhotonSeg s, CylIntersection i, float extinction, vec4 dir ){
  vec3 beam_dir        = glm::normalize( vec3( s.end ) - vec3( s.start ) );
  vec3 camera_dir      = glm::normalize( vec3( dir ) );
  float cos_theta      = glm::dot( beam_dir, -camera_dir );
  float length_const   = glm::length( s.orig_start - s.end );

  float integrand = 0;
  float dt_c      = 0.001;

  for( float tc=i.tc_minus; tc<i.tc_plus; tc = tc + dt_c ){
    float tb  = i.tb_minus - ( abs( cos_theta ) * ( tc - i.tc_minus ) );
    float current_r   = ( s.radius / length_const ) * tb;
    float constant = Transmittance( tb, extinction );
    float transmitted = Transmittance( tc, extinction ) * constant
                        * ( scattering_c / pow( current_r, 2 ) );
    if( transmitted > 1e-6 ){
      integrand += transmitted;
    }
  }
  return integrand;
}

float Integral_721( PhotonSeg s, CylIntersection i, float extinction, vec4 dir ){
  vec3 beam_dir        = glm::normalize( vec3( s.end ) - vec3( s.start ) );
  vec3 camera_dir      = glm::normalize( vec3( dir ) );
  float cos_theta      = glm::dot( beam_dir, -camera_dir );

  float integrand = 0;
  float dt_c      = 0.001;

  for( float tc=i.tc_minus; tc<i.tc_plus; tc = tc + dt_c ){
    float tb  = i.tb_minus - ( abs( cos_theta ) * ( tc - i.tc_minus ) );
    float constant = Transmittance( tb, extinction );
    float transmitted = Transmittance( tc, extinction ) * constant;
    if( transmitted > 1e-6 ){
      integrand += transmitted;
    }
  }
  return integrand;
}

float Integral_722_ada( PhotonSeg s, CylIntersection i, float extinction, vec4 dir ){
  vec3 beam_dir        = glm::normalize( vec3( s.end ) - vec3( s.start ) );
  vec3 camera_dir      = glm::normalize( vec3( dir ) );
  float cos_theta      = glm::dot( beam_dir, -camera_dir );
  float integrand      = 0;
  float dt_b           = 0.001;
  float length_const   = glm::length( s.orig_start - s.end );
  // cout << "tb min: " << i.tb_minus << endl;
  // cout << "tb max: " << i.tb_plus << endl;

  for( float tb=i.tb_minus; tb>i.tb_plus; tb = tb - dt_b ){
    // NOTE: I am using the wrong radius
    float current_r   = ( s.radius / length_const ) * tb;
    if( current_r < 1e-6 ){
      continue;
    }
    float tc          = i.tc_minus - ( abs( cos_theta ) * ( tb - i.tb_minus ) );
    float constant    = Transmittance( tc, extinction );
    float transmitted = Transmittance( tb, extinction );
    float tra_radius  = transmitted * constant *
                        ( scattering_c / pow( current_r, 2 ) );
    if( tra_radius > 1e-6 ){
      integrand += tra_radius;
    }
  }
  // cout << "Integrand: " << integrand << endl;
  return integrand;
}

float Integral_73( PhotonSeg s, CylIntersection i, float extinction, vec4 dir  ){
  float transmitted= 0;
  vec3 x_1         = vec3( s.start );   vec3 x_3 = i.exit_point;
  vec3 x_2         = vec3( s.end );     vec3 x_4 = i.entry_point;
  vec3 a           = glm::normalize( x_2 - x_1 );
  vec3 b           = glm::normalize( vec3( dir ) );
  vec3 c           = x_1 - x_3;
  float a_dot_b    = glm::dot( a, b );
  float b_dot_c    = glm::dot( b, c );
  float a_dot_c    = glm::dot( a, c );
  float a_dot_a    = glm::dot( a, a );
  float b_dot_b    = glm::dot( b, b );
  float denominator= ( a_dot_a * b_dot_b ) - ( a_dot_b * a_dot_b );

  if( a_dot_a != 0 && b_dot_b !=0 && denominator != 0 ){
    float d          = ( ( -a_dot_b * b_dot_c ) + ( a_dot_c * b_dot_b ) ) / denominator;
    float e          = ( ( a_dot_b * a_dot_c ) - ( b_dot_c * a_dot_a ) ) / denominator;
    vec3 intersect_d = x_1 + ( a * d );
    vec3 intersect_e = x_3 + ( e * d );

    float t_bc       = glm::length( intersect_d - vec3( s.orig_start ) );

    float t_cb       = glm::length( intersect_e );

    transmitted= Transmittance( t_bc, extinction ) * Transmittance( t_cb, extinction );
  }

  float sin_theta  = glm::length( glm::cross( a, b ) );

  // cout << ( transmitted / sin_theta ) << endl;

  return ( transmitted / sin_theta );
}


// TODO: Check if the limit is working correctly
void BeamRadiance( screen* screen, vec4 start, vec4 dir, const Intersection& limit, Node* parent,
                   vec3& current, vector<PhotonBeam>& beams ){
  vec4 hit;

  Node* left     = parent->left;
  Node* right    = parent->right;

  mat4 matrix;  TransformationMatrix( matrix );
  float max_distance  = glm::length( limit.position - start );

  if( left != NULL){
    AABB box_left  = left->aabb;
    // DrawBoundingBox( screen, box_left );
    if( HitBoundingBox( box_left, start, dir, hit ) ){
      // Pixel h; Vertex vh; vh.position = hit;
      // VertexShader( vh, h );
      // PixelShader( screen, h.x, h.y, vec3(1,1,1) );
      float hit_distance = glm::length( hit - start );
      if( hit_distance <= max_distance ){
        PhotonSeg segments[2];
        segments[0] = left->segments[0];
        segments[1] = left->segments[1];
        for( unsigned int i = 0; i < sizeof(segments)/sizeof(segments[0]); i++ ){
          PhotonSeg seg = segments[i];
          if( seg.id != -1 ){
            CylIntersection intersect;
            if( seg.ada_width ){
              // cout << seg.radius << endl;
              if( HitCone( start, dir, seg, intersect ) ){
                if( intersect.valid ){

                  float _int     = Integral_721_ada( seg,
                                                     intersect,
                                                     extinction_c,
                                                     dir );

                  float phase_f  = 1 / ( 4 * PI );

                  PhotonBeam beam = beams[ seg.id ];
                  current        += beam.energy * phase_f * _int;
                }
              }
            } else if( HitCylinder( start, dir, seg, intersect ) ){
              if( intersect.valid ){
                float _int     = Integral_721( seg,
                                               intersect,
                                               extinction_c,
                                               dir );

                float phase_f  = 1 / ( 4 * PI );
                float rad      = scattering_c / ( pow( seg.radius, 2 ) );

                PhotonBeam beam = beams[ seg.id ];
                // current        += beam.energy * phase_f * rad * _int;

                float error_1   = glm::length( intersect.entry_point -
                                               vec3( limit.position ) );
                float error_2   = glm::length( intersect.exit_point -
                                               vec3( limit.position ) );
                if( ( ( error_1 || error_2 ) <= ( seg.radius + 0.01 ) ) &&
                  beam.absorbed ){
                  float transmitted = Transmittance( max_distance, extinction_c );
                  current        += transmitted * beam.energy;
                } else {
                  current        += beam.energy * phase_f * rad * _int;
                }
              }
            }
          }
        }
        if( left->left != NULL || left->right != NULL ){
          BeamRadiance( screen, start, dir, limit, left, current, beams );
        }
      }
    }
  }

  if( right != NULL ){
    AABB box_right = right->aabb;
    if( HitBoundingBox( box_right, start, dir, hit ) ){
      float hit_distance = glm::length( hit - start );
      if( hit_distance < max_distance ){
        // Hits the right box at some point
        PhotonSeg segments[2];
        segments[0] = right->segments[0];
        segments[1] = right->segments[1];
        for( unsigned int i = 0; i < sizeof(segments)/sizeof(segments[0]); i++ ){
          PhotonSeg seg = segments[i];
          if( seg.id != -1 ){
            CylIntersection intersect;
            if( seg.ada_width ){
              if( HitCone( start, dir, seg, intersect ) ){
                if( intersect.valid ){
                  float _int     = Integral_721_ada( seg,
                                                     intersect,
                                                     extinction_c,
                                                     dir );

                  float phase_f  = 1 / ( 4 * PI );

                  PhotonBeam beam = beams[ seg.id ];
                  current        += beam.energy * phase_f * _int;
                }
              }
            } else if( HitCylinder( start, dir, seg, intersect ) ){
              if( intersect.valid ){
                float _int     = Integral_721( seg,
                                               intersect,
                                               extinction_c,
                                               dir );
                float phase_f  = 1 / ( 4 * PI );
                float rad      = scattering_c / ( pow( seg.radius, 2 ) );

                PhotonBeam beam = beams[ seg.id ];
                // current        += beam.energy * phase_f * rad * _int;

                float error_1   = glm::length( intersect.entry_point -
                                               vec3( limit.position ) );
                float error_2   = glm::length( intersect.exit_point -
                                               vec3( limit.position ) );

                if( ( ( error_1 || error_2 ) <= ( seg.radius + 0.01 ) ) &&
                  beam.absorbed ){
                  float transmitted = Transmittance( max_distance, extinction_c );
                  current        += transmitted * beam.energy;
                } else {
                  current        += beam.energy * phase_f * rad * _int;
                }
              }
            }
          }
        }
        if( right->left != NULL || right->right != NULL ){
          BeamRadiance( screen, start, dir, limit, right, current, beams );
        }
      }
    }
  }
}

vec3 DirectLight( const Intersection& i ){
  mat4 matrix;  TransformationMatrix(matrix);
  vec4 light_location = matrix * light_position;
  vec4 position       = i.position;

  vec3 radius = vec3 ( light_location ) - vec3( position );
  float A = 4 * PI * glm::dot( radius, radius );
  vec4 normal = vec4( triangles[i.index].normal );
  if( i.water ) normal = waves[i.index-triangles.size()].normal;
  float r_dot_n = glm::dot( glm::normalize( radius ), glm::normalize( vec3( normal ) ) );
  r_dot_n = fmax( r_dot_n, 0.0f );

  vec4 direction = glm::normalize( vec4( radius, 1.0f ) );
  vec4 start = position + 0.001f * normal;
  Intersection c_i;
  ClosestIntersection( start, direction, c_i, matrix, triangles );

  if( c_i.distance < glm::length( start - light_location ) ){
    r_dot_n = 0;
  }

  return ( light_power * r_dot_n ) / A;
}

void TransformationMatrix(glm::mat4& M){
	  M[0][0] = cos(theta.y) * cos(theta.z);
    M[0][1] = cos(theta.y) * sin(theta.z);
    M[0][2] = -sin(theta.y);
    M[0][3] = 0;

    M[1][0] = (-cos(theta.x)*sin(theta.z)) + (sin(theta.x)*sin(theta.y)*cos(theta.z));
    M[1][1] = (cos(theta.x)*cos(theta.z)) + (sin(theta.x)*sin(theta.y)*sin(theta.z));
    M[1][2] = sin(theta.x)*cos(theta.y);
    M[1][3] = 0;

    M[2][0] = (sin(theta.x)*sin(theta.z)) + (cos(theta.x)*sin(theta.y)*cos(theta.z));
    M[2][1] = (-sin(theta.x)*cos(theta.z)) +(cos(theta.x)*sin(theta.y)*sin(theta.z));
    M[2][2] = cos(theta.x)*cos(theta.y);
    M[2][3] = 0;

    M[3][0] = -camera.x;
    M[3][1] = -camera.y;
    M[3][2] = -camera.z;
    M[3][3] = 1;
  }

bool UserInput(){

  SDL_Event e;
  while( SDL_PollEvent( &e ) )
  {
    if ( e.type == SDL_QUIT )
    {
      return false;
    }
    else if ( e.type == SDL_KEYDOWN )
    {
      int key_code = e.key.keysym.sym;
      switch(key_code)
      {
        // Rotation
        case SDLK_UP:
        theta.x += PI/180;
    		break;
    	  case SDLK_DOWN:
        theta.x -= PI/180;
    		break;
    	  case SDLK_LEFT:
    		theta.y += PI/180;
    		break;
    	  case SDLK_RIGHT:
    		theta.y -= PI/180;
    		break;

        // Move forward/back/left/right
        case SDLK_w:
        camera.z += 0.1;
        break;
        case SDLK_s:
        camera.z -= 0.1;
        break;
        case SDLK_a:
        camera.x -= 0.1;
        break;
        case SDLK_d:
        camera.x += 0.1;
        break;

        // Move the light
        case SDLK_l:
        light_position.x += 0.1;
        break;
        case SDLK_j:
        light_position.x -= 0.1;
        break;
        case SDLK_o:
        light_position.y -= 0.1;
        break;
        case SDLK_u:
        light_position.y += 0.1;
        break;
        case SDLK_k:
        light_position.z -= 0.1;
        break;
        case SDLK_i:
        light_position.z += 0.1;
        break;

        // Reset state
        case SDLK_r:
        camera = vec4( 0, 0, -3.00, 1 );
        theta = vec3( 0.0, 0.0, 0.0 );
        light_position = vec4(0,-0.5,-0.7,1);
        break;

    	  case SDLK_ESCAPE:
    		/* Move camera quit */
    		return false;
      }
    }
  }
  return true;
}
