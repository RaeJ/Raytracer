#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "CastPhotons.h"
#include "Analysis.h"

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
                   glm::dvec3& current,
                   vector<PhotonBeam>& beams
                 );
double Integral_721( PhotonSeg s,
                    CylIntersection i,
                    float extinction,
                    vec4 dir  );
float Integral_722( screen* screen,
                    const vec4 start,
                    const vec4 limit,
                    PhotonSeg s,
                    CylIntersection i,
                    float extinction,
                    vec4 dir );
double Integral_73( PhotonSeg s,
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
    }
    vec4 center = matrix * vec4( 0.8, 1, -0.5, 1 );
    float radius = 0.2;
    BasicIntersection intersect2;
    if( SphereIntersection( vec3( start ), vec3( direction ), radius, vec3( center ), intersect2 ) ){
      colour = vec3( 0.3, 0, 0.7 );
    }
}

int main( int argc, char* argv[] )
{
  srand (time(NULL));
  if( RUN_ANALYSIS ){
    RunAnalysis();
  }

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


void Draw( screen* screen, vector<PhotonBeam> beams, vector<PhotonSeg>& items )
{
  // mat4 inverse_matrix = glm::inverse( root_matrix );
  mat4 matrix;

  TransformationMatrix( matrix );

  for( int x = 0; x < (SCREEN_WIDTH * SSAA); x+=SSAA ) {
    for( int y = 0; y < (SCREEN_HEIGHT * SSAA); y+=SSAA ) {
      glm::dvec3 current = vec3( 0, 0, 0 );
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

            float max_distance = glm::length( c_i.position - start );

            if( SHORT_VIEW ){
              float t_a = -( log( 1 - uniform( generator ) ) / extinction_c );
              if( t_a > 0 ){
                // TODO: Add in scattered radiance from beam direction change?
                max_distance   = fmin( t_a, max_distance );
                c_i.position   = start + ( direction * max_distance );
              }
            }

            BeamRadiance( screen, start, direction, c_i, root, current, beams );
          }
          // Testing( screen, start, direction, colour, current, beams, items, matrix );
        }
      }
      // PutPixelSDL( screen, x / SSAA, y / SSAA, colour / (float) SSAA );
      PutPixelSDL( screen, x / SSAA, y / SSAA, current / (double) SSAA );
    }
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
  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
  return( UserInput() );
}

double Integral_721_ada( PhotonSeg s, CylIntersection i, float extinction, vec4 dir ){
  vec3 beam_dir        = glm::normalize( vec3( s.end ) - vec3( s.start ) );
  vec3 camera_dir      = glm::normalize( vec3( dir ) );
  double cos_theta      = glm::dot( beam_dir, -camera_dir );
  double length_const   = glm::length( s.orig_start - s.end );

  double integrand = 0;
  double dt_c      = 0.001;

  for( double tc=i.tc_minus; tc<i.tc_plus; tc = tc + dt_c ){
    double tb  = i.tb_minus - ( abs( cos_theta ) * ( tc - i.tc_minus ) );
    double current_r   = ( s.radius / length_const ) * tb;
    double constant = Transmittance( tb, extinction );
    double transmitted = Transmittance( tc, extinction ) * constant
                        * ( scattering_c / pow( current_r, 2 ) );
    // if( transmitted > 1e-6 ){
    integrand += transmitted;
    // }
  }
  return integrand;
}

// TODO: Work out why it works for analytical but not integrated
double Integral_721( PhotonSeg s, CylIntersection i, float extinction, vec4 dir ){
  vec3 beam_dir        = glm::normalize( vec3( s.end ) - vec3( s.start ) );
  vec3 camera_dir      = glm::normalize( vec3( dir ) );
  float cos_theta      = glm::dot( beam_dir, -camera_dir );

  // double integrand = 0;
  // double dt_c      = 0.001;

  // for( double tc=i.tc_minus; tc<i.tc_plus; tc = tc + dt_c ){
  //   double tb  = i.tb_minus - ( abs( cos_theta ) * ( tc - i.tc_minus ) );
  //   double constant = Transmittance( tb, extinction );
  //   double transmitted = Transmittance( tc, extinction ) * constant;
  //   // if( transmitted > 1e-6 ){
  //   integrand += transmitted;
  //   // }
  // }
  double numerator = exp( -extinction * ( i.tc_minus - i.tc_plus ) *
                     ( abs( cos_theta ) - 1 ) ) - 1;
  double denominator = exp( extinction * ( i.tc_minus + i.tb_minus ) ) *
                       extinction * ( abs( cos_theta ) - 1 );
  return ( numerator / denominator );
}

// TODO: FIX
float Integral_722( screen* screen, const vec4 start, const vec4 limit,
  PhotonSeg s, CylIntersection i, float extinction, vec4 dir ){
  vec3 x_b             = vec3( s.start );
  vec3 x_e             = vec3( s.end );
  vec3 beam_dir        = glm::normalize( x_e - x_b );
  vec3 camera_dir      = glm::normalize( vec3( -dir ) );
  float cos_theta      = glm::dot( beam_dir, camera_dir );

  float tb_minus, tb_plus;
  if( abs( cos_theta ) <= 1e-6 ){
    tb_minus = 0.0f;
    tb_plus  = glm::length( vec3( limit ) - vec3( s.orig_start ) );
    PositionShader( screen, vec4(i.entry_point, 1.0f), vec3(0,1,0));
  } else {
    float t_min = glm::dot( ( i.entry_point - x_b ), camera_dir ) / cos_theta;
    vec3 p_min  = x_b + ( beam_dir * t_min );
    tb_minus    = glm::length( p_min - vec3( s.orig_start ) );

    float t_max = glm::dot( ( i.exit_point - x_b ), camera_dir ) / cos_theta;
    vec3 p_max  = x_b + ( beam_dir * t_max );
    tb_plus     = glm::length( p_max - vec3( s.orig_start ) );
  }

  float integrand      = 0;
  float dt_b           = 0.001;
  float tc_minus       = glm::length( i.entry_point - vec3( start ) );

  float begin    = tb_minus;
  float end      = tb_plus;
  if( begin > end ){
    tb_minus     = end;
    tb_plus      = begin;
    // tc_minus     = glm::length( i.exit_point - vec3( start ) );
  }
  // float numerator = exp( -extinction * ( tb_minus - tb_plus ) *
  //                   ( abs( cos_theta ) - 1 ) ) - 1;
  //
  // float denominator = exp( extinction * ( tb_minus + tc_minus ) ) *
  //                     extinction * ( abs( cos_theta ) - 1 );
  // if( tb_minus > tb_plus ){
  //   numerator   = exp( -extinction * ( tb_minus - tb_plus ) *
  //                     ( abs( cos_theta ) + 1 ) ) - 1;
  //   denominator = exp( extinction * ( -tb_minus + tc_minus ) ) *
  //                       extinction * ( abs( cos_theta ) - 1 );
  // }
  // // if( tb_minus > tb_plus ){
  // //   numerator   = exp( -extinction * ( tb_minus - tb_plus ) *
  // //                     ( abs( cos_theta ) - 1 ) ) - 1;
  // //   denominator = exp( extinction * ( tb_minus + tc_minus ) ) *
  // //                       -extinction * ( abs( cos_theta ) + 1 );
  // // }
  // if( abs( denominator ) > 1e-5 ){
  //   integrand = numerator / denominator;
  // }

  // if( glm::length( integrand ) > 10 ){
  //   cout << "Cos theta: " << cos_theta << endl;
  //   cout << "Numerator: " << numerator << endl;
  //   cout << "Denominator: " << denominator << endl;
  //   PositionShader( screen, vec4(i.entry_point, 1.0f), vec3(0,1,0));
  // }

  for( float tb=tb_minus; tb<tb_plus; tb = tb + dt_b ){
    float tc  = tc_minus - ( abs( cos_theta ) * ( tb - tb_minus ) );
    float constant = Transmittance( tc, extinction );
    float transmitted = Transmittance( tb, extinction ) * constant;
    if( transmitted > 1e-6 ){
      integrand += transmitted;
    }
  }

  return integrand;
}

double Integral_73( PhotonSeg s, CylIntersection i, float extinction, vec4 dir  ){
  double transmitted= 0;
  glm::dvec3 x_1    = vec3( s.start );   glm::dvec3 x_3 = i.exit_point;
  glm::dvec3 x_2    = vec3( s.end );     glm::dvec3 x_4 = i.entry_point;
  glm::dvec3 a      = glm::normalize( x_2 - x_1 );
  glm::dvec3 b      = glm::normalize( vec3( dir ) );
  glm::dvec3 c      = x_1 - x_3;
  double a_dot_b    = glm::dot( a, b );
  double b_dot_c    = glm::dot( b, c );
  double a_dot_c    = glm::dot( a, c );
  double a_dot_a    = glm::dot( a, a );
  double b_dot_b    = glm::dot( b, b );
  double denominator= ( a_dot_a * b_dot_b ) - ( a_dot_b * a_dot_b );
  // TODO: check distance is less than the radius

  if( a_dot_a != 0 && b_dot_b !=0 && denominator != 0 ){
    double d          = ( ( -a_dot_b * b_dot_c ) + ( a_dot_c * b_dot_b ) ) / denominator;
    double e          = ( ( a_dot_b * a_dot_c ) - ( b_dot_c * a_dot_a ) ) / denominator;
    glm::dvec3 intersect_d = x_1 + ( a * d );
    glm::dvec3 intersect_e = x_3 + ( e * d );

    double t_bc       = glm::length( intersect_d - glm::dvec3( s.orig_start ) );

    double t_cb       = glm::length( intersect_e );

    // cout << "Beam: " << t_bc << endl;
    // cout << "Camera: " << t_cb << endl;
    Vertex v0; v0.position = vec4( intersect_d, 1.0f );
    Vertex v1; v1.position = vec4( intersect_e, 1.0f );
    cout << "-------------------------------------------------------" << endl;
    cout << "( " << intersect_d.x << ", " << intersect_d.y << ", " << intersect_d.z << " )" << endl;
    cout << "( " << intersect_e.x << ", " << intersect_e.y << ", " << intersect_e.z << " )" << endl;
    cout << glm::length( intersect_e - intersect_d ) << endl;

    transmitted= Transmittance( t_bc, extinction ) * Transmittance( t_cb, extinction );
  }

  double sin_theta  = glm::length( glm::cross( a, b ) );

  // cout << ( transmitted / sin_theta ) << endl;

  return ( transmitted / sin_theta );
}


// TODO: Check if the limit is working correctly
void BeamRadiance( screen* screen, vec4 start, vec4 dir, const Intersection& limit, Node* parent,
                   glm::dvec3& current, vector<PhotonBeam>& beams ){
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

                  double _int     = Integral_721_ada( seg,
                                                     intersect,
                                                     extinction_c,
                                                     dir );

                  double phase_f  = 1 / ( 4 * PI );

                  PhotonBeam beam = beams[ seg.id ];
                  current        += beam.energy * phase_f * _int;
                }
              }
            } else if( HitCylinder( start, dir, seg, intersect ) ){
              if( intersect.valid ){
                if( ONE_DIMENSIONAL ){
                  double _int     = Integral_73( seg,
                                                intersect,
                                                extinction_c,
                                                dir );

                  double phase_f  = 1 / ( 4 * PI );
                  double rad      = scattering_c / seg.radius;
                  PhotonBeam beam = beams[ seg.id ];

                  current        += beam.energy * phase_f * rad * _int;
                } else {
                  double _int     = Integral_721( seg,
                                                 intersect,
                                                 extinction_c,
                                                 dir );

                  double phase_f  = 1 / ( 4 * PI );
                  double rad      = scattering_c / ( PI * pow( seg.radius, 2 ) );

                  PhotonBeam beam = beams[ seg.id ];
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

                  double _int     = Integral_721_ada( seg,
                                                     intersect,
                                                     extinction_c,
                                                     dir );

                  double phase_f  = 1 / ( 4 * PI );

                  PhotonBeam beam = beams[ seg.id ];
                  current        += beam.energy * phase_f * _int;
                }
              }
            } else if( HitCylinder( start, dir, seg, intersect ) ){
              if( intersect.valid ){
                if( ONE_DIMENSIONAL ){
                  double _int     = Integral_73( seg,
                                                intersect,
                                                extinction_c,
                                                dir );

                  double phase_f  = 1 / ( 4 * PI );
                  double rad      = scattering_c / seg.radius;
                  PhotonBeam beam = beams[ seg.id ];

                  current        += beam.energy * phase_f * rad * _int;
                } else {
                  double _int     = Integral_721( seg,
                                                 intersect,
                                                 extinction_c,
                                                 dir );

                  double phase_f  = 1 / ( 4 * PI );
                  double rad      = scattering_c / ( PI * pow( seg.radius, 2 ) );

                  PhotonBeam beam = beams[ seg.id ];
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
