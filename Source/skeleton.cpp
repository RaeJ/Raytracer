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

std::random_device rd;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
std::normal_distribution<> dis(0.0, 4 );

AABB rroot;
Node* root;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw( screen* screen, vector<PhotonBeam> beams, vector<PhotonSeg>& items );

void TransformationMatrix( glm::mat4& m );
void UserInput();
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
    } else if( ClosestIntersection( start, direction, c_i, matrix, triangles )  ){
      Triangle close = triangles[c_i.index];
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
  // CreateSurface( 10, -0.6, 1.004 );

  vector<PhotonBeam> beams;
  vector<PhotonSeg> items;

  LoadTestModel( triangles );

  cout << "Casting photons" << endl;
  mat4 matrix;  TransformationMatrix( matrix );
  rroot = CastPhotonBeams( PHOTON_NUMBER, beams, matrix, triangles );
  BoundPhotonBeams( beams, items, triangles );
  cout << "Beams size: " << beams.size() << "\n";
  cout << "Segment size: " << items.size() << "\n";
  root = newNode( rroot );
  cout << "Building tree" << endl;
  BuildTree( root, items );
  cout << "Calculating radiance" << endl;

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  while( NoQuitMessageSDL() )
    {
      Update();
      Draw( screen, beams, items );
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

void Draw( screen* screen, vector<PhotonBeam> beams, vector<PhotonSeg>& items )
{
  mat4 matrix;  TransformationMatrix(matrix);
  /* Clear buffer */
  // memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  // Drawing stage
  for( int x = 0; x < (SCREEN_WIDTH * SSAA); x+=SSAA ) {
    for( int y = 0; y < (SCREEN_HEIGHT * SSAA); y+=SSAA ) {
      vec3 current = vec3( 0, 0, 0 );
      vec3 colour  = vec3( 0, 0, 0 );
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
            // colour = close.colour;
            BeamRadiance( screen, start, direction, c_i, root, current, beams );
          }
          // Testing( screen, start, direction, colour, current, beams, items, matrix );

        }
      }
      // PutPixelSDL( screen, x / SSAA, y / SSAA, ( current / (float) SSAA  + indirect_light ) * colour );
      PutPixelSDL( screen, x / SSAA, y / SSAA, current / (float) SSAA );
    }
    // for( int i=0; i<beams.size(); i++ ){
    //   DrawBeam( screen, beams[i], vec3(1,0,1) );
    //   if( beams[i].ada_width ){
    //     float tmp_len = glm::length( beams[i].end - beams[i].start );
    //     PhotonBeam a; a.start = beams[i].start; a.end = a.start + ( beams[i].omega_u * tmp_len );
    //     DrawBeam( screen, a, vec3( 0, 0, 1) );
    //     PhotonBeam b; b.start = beams[i].start; b.end = b.start + ( beams[i].omega_v * tmp_len );
    //     DrawBeam( screen, b, vec3( 0, 0, 1) );
    //   }
    // }
    // for( int i=0; i<segments.size(); i++ ){
    //   PositionShader( screen, segments[i].start, vec3(1,0,1));
    //   PositionShader( screen, segments[i].end, vec3(0,1,1));
    // }

    SDL_Renderframe(screen);
  }
}

/*Place updates of parameters here*/
void Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  // std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
  UserInput();
}

float Integral_721( PhotonSeg s, CylIntersection i, float extinction, vec4 dir ){
  vec3 beam_dir        = glm::normalize( vec3( s.end ) - vec3( s.start ) );
  vec3 camera_dir      = glm::normalize( vec3( dir ) );
  float cos_theta      = glm::dot( beam_dir, -camera_dir );

  float integrand = 0;
  float dt_c      = 0.01;

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

  for( float tb=i.tb_minus; tb<i.tb_plus; tb = tb + dt_b ){
    float current_r   = ( s.radius / length_const ) * tb;
    float tc          = i.tc_minus - ( abs( cos_theta ) * ( tb - i.tb_minus ) );
    float constant    = Transmittance( tc, extinction );
    float transmitted = Transmittance( tb, extinction );
    float tra_radius  = transmitted * constant *
                        ( scattering_c / pow( current_r, 2 ) );
    if( tra_radius > 1e-6 ){
      integrand += tra_radius;
    }
  }
  return integrand;
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

                  float _int     = Integral_722_ada( seg,
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
                current        += beam.energy * phase_f * rad * _int;
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
                  float _int     = Integral_722_ada( seg,
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
                current        += beam.energy * phase_f * rad * _int;
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
  float r_dot_n = glm::dot( glm::normalize( radius ), glm::normalize( vec3( normal ) ) );
  r_dot_n = max( r_dot_n, 0.0f );

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

  void UserInput(){
    const uint8_t* keystate = SDL_GetKeyboardState( 0 );

    if( keystate == NULL ) {
      printf("keystate = NULL!\n");
    }
    // Rotation
    if( keystate[SDL_SCANCODE_UP] ) {
      theta.x -= PI/180;
    }
    if( keystate[SDL_SCANCODE_DOWN] ) {
      theta.x += PI/180;
    }
    if( keystate[SDL_SCANCODE_LEFT] ) {
      theta.y += PI/180;
    }
    if( keystate[SDL_SCANCODE_RIGHT] ) {
      theta.y -= PI/180;
    }
    // Move forward/back/left/right
    if( keystate[SDL_SCANCODE_W] ) {
      camera.z += 0.1;
    }
    if( keystate[SDL_SCANCODE_S] ) {
      camera.z -= 0.1;
    }
    if( keystate[SDL_SCANCODE_A] ) {
      camera.x -= 0.1;
    }
    if( keystate[SDL_SCANCODE_D] ) {
      camera.x += 0.1;
    }
    // Move the light
    if( keystate[SDL_SCANCODE_I] ) {
      light_position.z += 0.1;
    }
    if( keystate[SDL_SCANCODE_K] ) {
      light_position.z -= 0.1;
    }
    if( keystate[SDL_SCANCODE_J] ) {
      light_position.x -= 0.1;
    }
    if( keystate[SDL_SCANCODE_L] ) {
      light_position.x += 0.1;
    }
    if( keystate[SDL_SCANCODE_U] ) {
      light_position.y += 0.1;
    }
    if( keystate[SDL_SCANCODE_O] ) {
      light_position.y -= 0.1;
    }

    // Reset state
    if( keystate[SDL_SCANCODE_R] ) {
      camera = vec4( 0, 0, -3.00, 1 );
      theta = vec3( 0.0, 0.0, 0.0 );
      light_position = vec4(0,-0.5,-0.7,1);;
    }
  }
