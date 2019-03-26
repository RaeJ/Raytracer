#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "CastPhotons.h"

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

/* ----------------------------------------------------------------------------*/
/* STRUCTS                                                                     */

struct CylIntersection
{
  bool valid;
  float tb_minus;
  float tb_plus;
  float tc_minus;
  float tc_plus;
  vec3 entry_point;
  vec3 exit_point;
  vec3 entry_normal;
  vec3 exit_normal;
};

/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */

vector<Triangle> triangles;

vec4 camera(0, 0, -3, 1.0);
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
bool HitCylinder( const vec4 start,
                  const vec4 dir,
                  const PhotonSeg& seg,
                  CylIntersection& intersection );
void TransformationMatrix( glm::mat4& m );
void UserInput();
vec3 DirectLight( const Intersection& i );
bool HitBoundingBox( AABB box, vec4 start, vec4 dir, vec4& hit );
void BeamRadiance( screen* screen,
                   vec4 start,
                   vec4 dir,
                   const Intersection& limit,
                   Node* parent,
                   vec3& current,
                   vector<PhotonBeam>& beams
                 );
float Integral_721( float tc_m, float tc_p, float tb_p, float extinction );
mat3 findRotationMatrix( vec3 current_dir,
                         vec3 wanted_dir );

// ------------------------------------------------------------------------- //

int main( int argc, char* argv[] )
{
  srand (time(NULL));
  CreateSurface( 10, -0.6, 1.004 );

  vector<PhotonBeam> beams;
  vector<PhotonSeg> items;

  LoadTestModel( triangles );

  cout << "Casting photons" << endl;
  mat4 matrix;  TransformationMatrix( matrix );
  rroot = CastPhotonBeams( 500, beams, matrix, triangles, camera );
  BoundPhotonBeams( beams, items, triangles );
  cout << "Beams size: " << beams.size() << "\n";
  cout << "Segment size: " << items.size() << "\n";
  root = newNode( rroot );
  cout << "Building tree" << endl;
  BuildTree( root, items );

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
            colour = close.colour;
            // BeamRadiance( screen, start, direction, c_i, root, current, beams );
          }
        }
        PutPixelSDL( screen, x / SSAA, y / SSAA, colour );
        // PutPixelSDL( screen, x / SSAA, y / SSAA, current / (float) SSAA );
      }
    }
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

float Integral_721( float tc_m, float tc_p, float tb_p, float extinction ){
  float integrand = 0;
  float dt_c      = 0.01;
  float constant  = Transmittance( tb_p, extinction_c );

  for( float tc=tc_m; tc<tc_p; tc = tc + dt_c ){
    integrand += ( Transmittance( tc, extinction_c ) );
  }
  return integrand * constant ;
}

mat3 findRotationMatrix( vec3 c_unit_dir, vec3 w_unit_dir ){
  vec3 v   = glm::cross( c_unit_dir, w_unit_dir );
  float c  = glm::dot( c_unit_dir, w_unit_dir );

  mat3 identity( 1.0f );

  mat3 v_x;
  v_x[0][0] = 0;    v_x[1][0] = -v.z;  v_x[2][0] = v.y;
  v_x[0][1] = v.z;  v_x[1][1] = 0;     v_x[2][1] = -v.x;
  v_x[0][2] = -v.y; v_x[1][2] = v.x;   v_x[2][2] = 0;

  mat3 R = identity + v_x + ( ( v_x * v_x) * ( 1 / ( 1 + c ) ) );

  return R;
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
            if( HitCylinder( start, dir, seg, intersect ) ){
              if( intersect.valid ){
                float _int     = Integral_721( intersect.tc_minus,
                                               intersect.tc_plus,
                                               intersect.tb_plus,
                                               extinction_c );

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
            if( HitCylinder( start, dir, seg, intersect ) ){
              if( intersect.valid ){
                float _int     = Integral_721( intersect.tc_minus,
                                               intersect.tc_plus,
                                               intersect.tb_plus,
                                               extinction_c );
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

bool HitBoundingBox( AABB box, vec4 start, vec4 dir, vec4& hit ){
  const int DIMS = 3; int RIGHT=0; int LEFT=1; int MIDDLE=2;
  int quadrant[DIMS];
  bool inside = true;
  float min_b[DIMS] = { box.min.x, box.min.y, box.max.z };
  float max_b[DIMS] = { box.max.x, box.max.y, box.min.z };
  float candidate_plane[DIMS];
  float distances[DIMS];
  float position[DIMS];

  dir                   = glm::normalize( dir );
  float origin[DIMS]    = {start.x, start.y, start.z};
  float direction[DIMS] = {dir.x,   dir.y,   dir.z};

  for ( int i=0; i<DIMS; i++ ){
    if( origin[i] < min_b[i] ){
      quadrant[i]        = LEFT;
      candidate_plane[i] = min_b[i];
      inside             = false;
    } else if ( origin[i] > max_b[i] ){
      quadrant[i]        = RIGHT;
      candidate_plane[i] = max_b[i];
      inside             = false;
    } else {
      quadrant[i]        = MIDDLE;
    }
  }

  if( inside ){
    hit = vec4( start.x, start.y, start.z, 1.0f );
    return true;
  }

  for( int i=0; i<DIMS; i++ ){
    if(  direction[i] != 0 && quadrant[i] != MIDDLE ){
      distances[i] =  ( candidate_plane[i] - origin[i] ) / direction[i];
    } else {
      distances[i] = -1;
    }
  }

  int largest = 0;
  for( int i=0; i<DIMS; i++ ){
    if( distances[largest] < distances[i] ){
      largest = i;
    }
  }

  if( distances[largest] < 0 ) return false;
  for( int i=0; i<DIMS; i++ ){
    if ( largest != i ){
      position[i] = origin[i] + ( distances[largest] * direction[i] );
      if (position[i] < min_b[i] || position[i] > max_b[i]){

				return false;
      }
    } else {
      position[i] = candidate_plane[i];
    }
  }

  hit = vec4( position[0], position[1], position[2], 1.0f );
  return true;
}

bool HitCylinder( const vec4 start, const vec4 dir, const PhotonSeg& seg,
  CylIntersection& intersection ){
    bool first_inte = false;
    vec3 first_normal;
    vec3 second_normal;

    vec3 l_unit_dir = glm::normalize( vec3( dir ) );
    vec3 difference = vec3( seg.end - seg.start );
    vec3 w_unit_dir = glm::normalize( vec3( 0.0f,
                                            glm::length( difference ),
                                            0.0f ) );

    float S_inv     = 1/glm::length( difference ) ;

    vec3 c_unit_dir = difference * S_inv;

    mat3 R_inv      = findRotationMatrix( c_unit_dir, w_unit_dir );

    vec3 T_inv      = -( R_inv * vec3( seg.start ) * S_inv );

    vec3 origin_prime    = ( R_inv * ( vec3( start ) * S_inv  ) ) + T_inv;
    vec3 dir_prime       = R_inv * l_unit_dir;

    vec3 first_hit;
    vec3 second_hit;

    // Quadratic Formula
    float t0 = -1, t1 = -1;

    // a=xD2+yD2, b=2xExD+2yEyD, and c=xE2+yE2-1.
    float a = dir_prime[0] * dir_prime[0]
            + dir_prime[2] * dir_prime[2];

    float b = 2 * origin_prime[0] * dir_prime[0]
            + 2 * origin_prime[2] * dir_prime[2];

    float c = origin_prime[0] * origin_prime[0]
            + origin_prime[2] * origin_prime[2]
            - pow( seg.radius, 2 );

    float b24ac = b*b - 4*a*c;

    if( b24ac < 0 ) return false;

    float sqb24ac = sqrtf(b24ac);
    t0 = (-b + sqb24ac) / (2 * a);
    t1 = (-b - sqb24ac) / (2 * a);

    if (t0>t1) {float tmp = t0;t0=t1;t1=tmp;}

    float y0 = origin_prime[1] + t0 * dir_prime[1];
    float y1 = origin_prime[1] + t1 * dir_prime[1];
    if ( y0 < 0 )
    {
      if( y1 < 0 ) return false;
      else
      {
        // hit the cap
        float th1 = t0 + (t1-t0) * y0 / (y0-y1);
        if ( th1 <= 0 ) return false;
        first_hit = origin_prime + ( dir_prime * th1 );
        first_inte = true;
        first_normal = vec3(0, -1, 0);

        if( y1 <= 1 ){
          second_hit = origin_prime + ( dir_prime * t1 );
          second_normal = glm::normalize( vec3( second_hit.x, 0, second_hit.z) );
        } else {
          float th2 = t1 + (t1-t0) * (y1-1) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
            second_normal = vec3(0, -1, 0);
          }
        }
      }
    }
    else if ( y0 >= 0 && y0 <= 1 )
    {
      // hit the cylinder bit
      if( t0 <= 0 ) return false;
      first_hit = origin_prime + ( dir_prime * t0 );
      first_inte = true;
      first_normal = glm::normalize( vec3( first_hit.x, 0, first_hit.z) );

      if( y1 >= 0 ){
        if( y1 <= 1 ){
          second_hit = origin_prime + ( dir_prime * t1 );
          second_normal = glm::normalize( vec3( -second_hit.x, 0, -second_hit.z) );
        } else {
          float th2 = t1 + (t1-t0) * (y1-1) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
            second_normal = vec3(0, -1, 0);
          }
        }
      } else {
        float th2 = t0 + (t1-t0) * (y0) / (y0-y1);
        if (th2>0){
          second_hit = origin_prime + ( dir_prime * th2 );
          second_normal = vec3(0, 1, 0);
        }
      }
    }
    else if ( y0 > 1 )
    {
      if ( y1 > 1 ) return false;
      else {
        // hit the cap
        float th = t0 + (t1-t0) * (y0-1) / (y0-y1);
        if( th <= 0 ) return false;
        first_hit = origin_prime + ( dir_prime * th );
        first_inte = true;
        first_normal = vec3(0, 1, 0);
        if( y1 >= 0 ){
          second_hit = origin_prime + ( dir_prime * t1 );
          second_normal = glm::normalize( vec3( -second_hit.x, 0, -second_hit.z) );
        } else {
          float th2 = t0 + (t1-t0) * (y0) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
            second_normal = vec3(0, 1, 0);
          }
        }
      }
    }
    if( ( t0 > 0 ) && ( t1 > 0 ) && ( t1 > t0 ) ){
      intersection.valid       = true;
      intersection.tb_minus    = first_hit.y / S_inv;
      intersection.tb_plus     = second_hit.y / S_inv;
      intersection.tc_minus    = t0 / S_inv;
      intersection.tc_plus     = t1 / S_inv;
      intersection.entry_point = ( glm::inverse( R_inv ) * ( first_hit - T_inv ) ) / S_inv;
      intersection.exit_point  = ( glm::inverse( R_inv ) * ( second_hit - T_inv ) ) / S_inv;
      intersection.entry_normal= ( glm::inverse( R_inv ) * ( first_normal ) );
      intersection.exit_normal = ( glm::inverse( R_inv ) * ( second_normal ) );
    } else {
      intersection.valid       = false;
    }
    return first_inte;
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
