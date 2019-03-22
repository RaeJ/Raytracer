#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <stdlib.h>
#include "rasteriser.h"
#include <math.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

#define BOUNCES 10

/* ----------------------------------------------------------------------------*/
/* STRUCTS                                                                     */

struct PhotonBeam
{
  vec4 start;
  vec4 end;
  float offset;
  float radius;
  vec3 energy;
  int index_hit;
};

struct PhotonSeg
{
  vec4 start;
  vec4 end;
  vec4 mid;
  vec4 min;
  vec4 max;
  float radius;
  int id;
};

struct CylIntersection
{
  float tb_minus;
  float tb_plus;
  float tc_minus;
  float tc_plus;
};

struct Node
{
  AABB aabb;
  PhotonSeg segments[2];
  struct Node *left;
  struct Node *right;
};

struct Node* newNode( AABB data )
{
  // Allocate memory for new node
  struct Node* node = (struct Node*)malloc(sizeof(struct Node));

  // Assign data to this node
  node->aabb = data;
  // node->segments = vector<PhotonSeg>();
  PhotonSeg init1;  PhotonSeg init2;
  init1.id = -1;    init2.id = -1;
  node->segments[0] = init1;
  node->segments[1] = init2;

  // Initialize left and right children as NULL
  node->left = NULL;
  node->right = NULL;
  return(node);
}

struct Intersection
{
  vec4 position;
  float distance;
  int index;
};

/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */

vector<Triangle> triangles;
vector<PhotonSeg> segments;

float m = std::numeric_limits<float>::max();

vec4 camera(0, 0, -3, 1.0);
vec3 theta( 0.0, 0.0, 0.0 );
// vec4 light_position(0,-1.2,-0.5,1);
vec4 light_position(0,-0.8,-0.7,1);
vec3 light_power = 0.1f * vec3( 1, 1, 1 );
vec3 indirect_light = 0.5f*vec3( 1, 1, 1 );

std::random_device rd;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
std::normal_distribution<> dis(0.0, 4 );

unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::mt19937 generator (seed);
std::uniform_real_distribution<double> uniform(0.0, 1.0);

AABB rroot;
Node* root;

float absorption_c = 0.035;
float scattering_c = 0.005;
float extinction_c = absorption_c + scattering_c;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw( screen* screen, vector<PhotonBeam> beams, vector<PhotonSeg>& items );
bool ClosestIntersection(
  vec4 start,
  vec4 dir,
  Intersection & closestIntersections
);
bool HitCylinder( screen* screen,
                  const vec4 start,
                  const vec4 dir,
                  const PhotonSeg& seg,
                  CylIntersection& intersection );
void TransformationMatrix( glm::mat4& m );
void UserInput();
vec3 DirectLight( const Intersection& i );
AABB CastPhotonBeams( int number, vector<PhotonBeam>& beams );
vec4 FindDirection( vec4 origin, vec4 centre, float radius );
void DrawBeam( screen* screen, PhotonBeam& b, vec3 colour );
void DrawBoundedBeams( screen* screen, vector<AABB> items );
void DrawTree( Node* parent, screen* screen );
void BuildTree( Node* parent, vector<PhotonSeg>& child );
void BoundPhotonBeams( vector<PhotonBeam>& beams, vector<PhotonSeg>& items );
bool HitBoundingBox( AABB box, vec4 start, vec4 dir, vec4& hit );
void BeamRadiance( screen* screen,
                   vec4 start,
                   vec4 dir,
                   vec4& limit,
                   Node* parent,
                   vec3& current,
                   vector<PhotonBeam>& beams
                 );
void CastBeam( int bounce,
               vec3 energy,
               vec4 origin,
               vec4 direction,
               vec4& min_point,
               vec4& max_point,
               vector<PhotonBeam>& beams,
               float offset,
               float radius
             );
float Transmittance( float length, float ext );
float Integral_722( float tc_m, float tc_p, float tb_p, float extinction );
mat3 findRotationMatrix( vec3 current_dir,
                         vec3 wanted_dir );
void Testing( screen* screen, vec4 start, vec4 dir, Node* parent, vec3 current );

void Testing( screen* screen, vec4 start, vec4 dir, Node* parent, vec3 current ){
  mat4 matrix;  TransformationMatrix( matrix );
  mat4 inv = glm::inverse( matrix );

  AABB box = parent->aabb;
  DrawBoundingBox( screen, box );
  vec4 hit;
  if( HitBoundingBox( box, start, dir, hit ) ){
    PhotonBeam b; b.start = inv * start; b.end = hit;
    DrawBeam( screen, b, vec3(0,0,0) );
  }
}

int main( int argc, char* argv[] )
{
  srand (time(NULL));

  vector<PhotonBeam> beams;
  vector<PhotonSeg> items;

  LoadTestModel( triangles );

  cout << "Casting photons" << endl;
  rroot = CastPhotonBeams( 500, beams );
  BoundPhotonBeams( beams, items );
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
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  // Drawing stage
  for( int x = 0; x < (SCREEN_WIDTH * SSAA); x+=SSAA ) {
    for( int y = 0; y < (SCREEN_HEIGHT * SSAA); y+=SSAA ) {
      vec3 current = vec3( 0, 0, 0 );
      for( int i = 0; i<SSAA; i++ ){
        for( int j = 0; j<SSAA; j++ ){
          float x_dir = ( x + i ) - ( (SCREEN_WIDTH * SSAA) / (float) 2 );
          float y_dir = ( y + j ) - ( (SCREEN_HEIGHT * SSAA) / (float) 2);

          vec4 direction = vec4( x_dir, y_dir, focal, 1.0);
          vec4 start = vec4( 0, 0, 0, 1 );
          Intersection c_i;

          direction = glm::normalize( direction );
          if( ClosestIntersection( start, direction, c_i ) ){
            Triangle close = triangles[c_i.index];
            BeamRadiance( screen, start, direction, c_i.position, root, current, beams );

            if( current.x > 0.001 ){
              // PutPixelSDL( screen, x / SSAA, y / SSAA, current * close.colour / (float) SSAA );
              PutPixelSDL( screen, x / SSAA, y / SSAA, current / (float) SSAA );
            }
          }
        }
      }
    }
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

void BuildTree( Node* parent, vector<PhotonSeg>& child ){
  if( child.size() <= 2 ) {
    for( int i=0; i<child.size(); i++ ){
      // parent->segments.push_back( child[i] );
      parent->segments[i] = child[i];
      segments.push_back( child[i] );
    }
    return;
  }
  vec4 diff     = abs( parent->aabb.max - parent->aabb.min );
  vec4 mid      = parent->aabb.mid;

  vector<PhotonSeg> l;
  vector<PhotonSeg> r;

  // TODO: Note, if the photon segment lies on a boundary it is ignored
  if( ( diff.x > diff.y ) && ( diff.x > diff.z ) ){
    for( int i=0; i<child.size(); i++ ){
      PhotonSeg box = child[i];
      if( box.mid.x < mid.x ){
        l.push_back( box );
      }  else if( box.mid.x > mid.x ) {
        r.push_back( box );
      } else {
        // TODO: what to do here?
        parent->segments[i%2] = child[i];
        // segments.push_back( child[i] );
      }
    }
  } else if( diff.y > diff.z ){
    for( int i=0; i<child.size(); i++ ){
      PhotonSeg box = child[i];
      if( box.mid.y < mid.y ){
        l.push_back( box );
      }  else if( box.mid.y > mid.y ){
        r.push_back( box );
      } else {
        // TODO: what to do here?
        parent->segments[i%2] = child[i];
        // segments.push_back( child[i] );
      }
    }
  } else {
    for( int i=0; i<child.size(); i++ ){
      PhotonSeg box = child[i];
      if( box.mid.z > mid.z ){
        l.push_back( box );
      } else if( box.mid.z < mid.z ){
        r.push_back( box );
      } else {
        // TODO: what to do here?
        parent->segments[i%2] = child[i];
        // segments.push_back( child[i] );
      }
    }
  }
  vec4 min = vec4( m, m, -m, 1 ); vec4 max = vec4( -m, -m, m, 1 );
  int l_size = l.size();
  if( l_size != 0 ){
    for( int i=0; i<l_size; i++ ){
      max.x = fmax( l[i].min.x, fmax( l[i].max.x, max.x ) );
      max.y = fmax( l[i].min.y, fmax( l[i].max.y, max.y ) );
      max.z = fmin( l[i].min.z, fmin( l[i].max.z, max.z ) );
      min.x = fmin( l[i].min.x, fmin( l[i].max.x, min.x ) );
      min.y = fmin( l[i].min.y, fmin( l[i].max.y, min.y ) );
      min.z = fmax( l[i].min.z, fmax( l[i].max.z, min.z ) );
    }

    AABB left_child;
    left_child.min = vec4( min.x, min.y, min.z, 1.0f );
    left_child.max = vec4( max.x, max.y, max.z, 1.0f );
    left_child.mid = ( min + max ) / 2.0f;
    Node *left_node = newNode( left_child );
    parent->left = left_node;
    BuildTree( parent->left, l );
  } else {
    parent -> left = NULL;
  }

  min = vec4( m, m, -m, 1 ); max = vec4( -m, -m, m, 1 );
  int r_size = r.size();
  if( r_size != 0 ){
    for( int i=0; i<r_size; i++ ){
      max.x = fmax( r[i].max.x, fmax( r[i].min.x, max.x ) );
      max.y = fmax( r[i].max.y, fmax( r[i].min.y, max.y ) );
      max.z = fmin( r[i].max.z, fmin( r[i].min.z, max.z ) );
      min.x = fmin( r[i].max.x, fmin( r[i].min.x, min.x ) );
      min.y = fmin( r[i].max.y, fmin( r[i].min.y, min.y ) );
      min.z = fmax( r[i].max.z, fmax( r[i].min.z, min.z ) );
    }
    AABB right_child;
    right_child.min = vec4( min.x, min.y, min.z, 1.0f );
    right_child.max = vec4( max.x, max.y, max.z, 1.0f );
    right_child.mid = ( min + max ) / 2.0f;
    Node *right_node = newNode( right_child );
    parent->right = right_node;
    BuildTree( parent->right, r );
  } else {
    parent -> right = NULL;
  }

}

// TODO: Fix this function at some point
void BoundPhotonBeams( vector<PhotonBeam>& beams, vector<PhotonSeg>& items ){
  vec3 x_dir     = glm::normalize( vec3( 1, 0, 0 ) );
  vec3 y_dir     = glm::normalize( vec3( 0, 1, 0 ) );

  float bound    = sqrt( 2 ) / 2;

  for( int i=0; i<beams.size(); i++ ){
    PhotonBeam b = beams[i];

    vec4 start   = b.start;
    vec4 end     = b.end;

    vec3 diff    = vec3( end - start );
    vec3 dir     = glm::normalize( diff );
    float length = glm::length( end - start );
    float j=0;

    float cos_x  = glm::dot( abs( dir ), x_dir );
    float cos_y  = glm::dot( abs( dir ), y_dir );

    Triangle tri = triangles[ b.index_hit ];

    // float step   = length;
    float step   = 0.2;

    vec3 prior   = vec3( start.x, start.y, start.z );

    while( j<length ){
      PhotonSeg beam_seg;
      vec3 next = prior + ( dir * step );

      beam_seg.start  = vec4( prior.x, prior.y, prior.z, 1.0f );

      if( next.x > end.x || next.y > end.y || next.z > end.z ){
        j = glm::length( length );
        beam_seg.end = vec4( end.x, end.y, end.z, 1.0f );
      } else {
        j = glm::length( next - vec3( start ) );
        beam_seg.end = vec4( next.x, next.y, next.z, 1.0f );
      }

      prior = vec3( beam_seg.end );

      beam_seg.radius = b.radius;
      beam_seg.id     = i;

      vec3 component  = vec3( 0, 0, 0 );
      if( cos_x < cos_y ){
        component.x = 0;
        component.y = beam_seg.radius / bound;
        component.z = beam_seg.radius / bound;
      } else {
        component.x = beam_seg.radius / bound;
        component.y = 0;
        component.z = beam_seg.radius / bound;
      }
      // cout << "component x: " << component.x << "\n";
      // cout << "component y: " << component.y << "\n";
      // cout << "component z: " << component.z << "\n";
      beam_seg.min    = vec4( ( fmin( beam_seg.start.x, beam_seg.end.x) ),
                              ( fmin( beam_seg.start.y, beam_seg.end.y) ),
                              ( fmax( beam_seg.start.z, beam_seg.end.z) ), 1.0f );
      beam_seg.max    = vec4( ( fmax( beam_seg.start.x, beam_seg.end.x) ),
                              ( fmax( beam_seg.start.y, beam_seg.end.y) ),
                              ( fmin( beam_seg.start.z, beam_seg.end.z) ), 1.0f );

      // if( length > 0.001 ){
      //   if( !( beam_seg.start.y == start.y ) ){
      //     beam_seg.min  = vec4( beam_seg.min.x - component.x,
      //                           beam_seg.min.y - component.y,
      //                           beam_seg.min.z + component.z, 1.0f );
      //   }
      //   if( !( ( vec3( beam_seg.end ) == vec3( end ) ) ) ){
      //     beam_seg.max  = vec4( beam_seg.max.x + component.x,
      //                           beam_seg.max.y + component.y,
      //                           beam_seg.max.z - component.z, 1.0f );
      //   } else {
      //     vec4 tri_normal = abs( tri.normal );
      //     if( tri_normal.x > ( tri_normal.y || tri_normal.z ) ){
      //       beam_seg.max  = vec4( beam_seg.max.x,
      //                             beam_seg.max.y + component.y,
      //                             beam_seg.max.z - component.z, 1.0f );
      //     } else if ( tri_normal.y > ( tri_normal.x || tri_normal.z ) ){
      //       beam_seg.max  = vec4( beam_seg.max.x + component.x,
      //                             beam_seg.max.y,
      //                             beam_seg.max.z - component.z, 1.0f );
      //     } else if ( tri_normal.z > ( tri_normal.x || tri_normal.y ) ){
      //       beam_seg.max  = vec4( beam_seg.max.x + component.x,
      //                             beam_seg.max.y + component.y,
      //                             beam_seg.max.z, 1.0f );
      //     }
      //   }
      // }



      beam_seg.mid    = ( beam_seg.min + beam_seg.max ) / 2.0f;

      items.push_back( beam_seg );
    }
  }
}

void DrawBoundedBeams( screen* screen, vector<AABB>& items ){
  for( int i = 0; i<items.size(); i++ ){
    DrawBoundingBox( screen, items[i] );
  }
}

void DrawTree( Node* parent, screen* screen ){
  AABB box = parent->aabb;
  DrawBoundingBox( screen, box );
  if( parent->left != NULL ){
    DrawTree( parent->left, screen );
  }
  if( parent->right != NULL ){
    DrawTree( parent->right, screen );
  }
}

void DrawBeam( screen* screen, PhotonBeam& b, vec3 colour ){
  Pixel proj1; Vertex v1; v1.position = b.start;
  Pixel proj2; Vertex v2; v2.position = b.end;
  VertexShader( v1, proj1 );
  VertexShader( v2, proj2 );
  DrawLine( screen, v1, v2, colour );
  // PixelShader( screen, proj1.x, proj1.y, vec3(0,1,0) );
  // PixelShader( screen, proj2.x, proj2.y, vec3(1,0,0) );
}

AABB CastPhotonBeams( int number, vector<PhotonBeam>& beams ){
  vec4 min_point = vec4( m, m, -m, 1 );
  vec4 max_point = vec4( -m, -m, m, 1 );

  mat4 matrix;  TransformationMatrix( matrix );
  vec4 origin    = matrix * light_position;
  vec4 centre    = vec4( origin.x, origin.y + 0.5, origin.z, 1.0f );
  float radius   = 0.2f;

  vec3 energy    = light_power;

  for( int i=0; i<number; i++ ){
    vec4 direction = FindDirection( origin, centre, radius );
    direction = glm::normalize( direction );

    float offset = uniform( generator ) * 0.1;
    // float r      = uniform( generator ) * 2;
    CastBeam( 0, energy, origin, direction, min_point, max_point, beams, offset, 0.05 );
  }

  AABB root;
  root.min = min_point;
  root.max = max_point;
  root.mid = ( min_point + max_point ) / 2.0f;

  return root;
}

void CastBeam( int bounce, vec3 energy, vec4 origin, vec4 direction,
               vec4& min_point, vec4& max_point, vector<PhotonBeam>& beams,
               float offset, float radius ){

   Intersection hit;
   if( ClosestIntersection( origin, direction, hit ) ){
     PhotonBeam beam;
     // TODO: Work out why the offset is not working as expected
     beam.offset    = offset;
     beam.radius    = radius;
     beam.energy    = energy;
     beam.index_hit = hit.index;

     beam.start     = origin + beam.offset;
     beam.end       = hit.position - beam.offset;

     beams.push_back( beam );

     max_point.x = fmax( beam.end.x, fmax( beam.start.x, max_point.x ) );
     max_point.y = fmax( beam.end.y, fmax( beam.start.y, max_point.y ) );
     max_point.z = fmin( beam.end.z, fmin( beam.start.z, max_point.z ) );
     min_point.x = fmin( beam.end.x, fmin( beam.start.x, min_point.x ) );
     min_point.y = fmin( beam.end.y, fmin( beam.start.y, min_point.y ) );
     min_point.z = fmax( beam.end.z, fmax( beam.start.z, min_point.z ) );

     float diff  = glm::length( vec3( hit.position - origin ) );

     if( bounce < BOUNCES ){
       // float rand            =  dis( gen ) * 0.01;
       int num               = bounce + 1;
       Triangle hit_triangle = triangles[hit.index];
       vec4 normal           = hit_triangle.normal;
       vec4 incident         = hit.position + direction;
       vec3 refl             = glm::reflect( vec3( incident ), vec3( normal ) );
       vec4 reflected        = vec4( glm::normalize( refl ), 1.0f );
       float transmitted     = Transmittance( diff, extinction_c );
       vec3 new_energy       = energy * transmitted;
       CastBeam( num, new_energy, hit.position, reflected,
                 min_point, max_point, beams, 0.0f, radius );
     }

     float scattered  = uniform( generator );
     if( scattered <= ( scattering_c / extinction_c ) ){
       float t_s         = diff * uniform( generator );
       // TODO: check incoming direction is already normalised
       vec4 start        = origin + ( t_s * direction );
       float the         = 2 * PI * uniform( generator );
       float phi         = acos(1 - 2 * uniform( generator ) );
       float x           = sin( phi ) * cos( the );
       float y           = sin( phi ) * sin( the );
       float z           = cos( phi );
       vec3 dir          = glm::normalize( vec3( x, y, z ) );
       vec4 dir_sample   = vec4( dir.x, dir.y, dir.z, 1.0f );
       float transmitted = Transmittance( t_s, extinction_c );
       vec3 new_energy   = energy * transmitted;
       CastBeam( bounce, new_energy, start, dir_sample,
                 min_point, max_point, beams, 0.0f, radius );
     }

   } else {
     cout << "Direction does not terminate\n";
   }

}

vec4 FindDirection( vec4 origin, vec4 centre, float radius ){
  float r = dis( gen );
  float incR = (rand() % 50) / (float) 100;
  float t = 2 * PI * r;
  float p = radius * r * incR;
  float x = centre.x + ( p * cos( t ) );
  float z = centre.z + ( p * sin( t ) );
  vec4 position( x, centre.y, z, 1.0f );

  return( position - origin );
}

float Transmittance( float length, float ext ){
  float power     = - ext * length;
  return exp( power );
}

float Integral_722( float tc_m, float tc_p, float tb_p, float extinction ){
  float integrand = 0;
  float dt_c      = 0.001;
  float constant  = Transmittance( tb_p, extinction_c );
  for( float tc=tc_m; tc<tc_p; tc = tc + dt_c ){
    integrand += ( Transmittance( tc, extinction_c ) * constant );
  }
  return integrand;
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
void BeamRadiance( screen* screen, vec4 start, vec4 dir, vec4& limit, Node* parent,
                   vec3& current, vector<PhotonBeam>& beams ){
  vec4 hit;

  Node* left     = parent->left;
  Node* right    = parent->right;

  mat4 matrix;  TransformationMatrix( matrix );
  float max_distance  = glm::length( limit - start );

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
            if( HitCylinder( screen, start, dir, seg, intersect ) ){
              float _int     = Integral_722( intersect.tc_minus,
                                             intersect.tc_plus,
                                             intersect.tb_plus,
                                             extinction_c );
              float phase_f  = 1 / ( 4 * PI );
              float rad      = scattering_c / ( pow( seg.radius, 2 ) );

              // current += beams[seg.id].energy * phase_f * rad * _int;
              current += beams[seg.id].energy * phase_f * rad;
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
            if( HitCylinder( screen, start, dir, seg, intersect ) ){
              float _int     = Integral_722( intersect.tc_minus,
                                             intersect.tc_plus,
                                             intersect.tb_plus,
                                             extinction_c );
              float phase_f  = 1 / ( 4 * PI );
              float rad      = scattering_c / ( pow( seg.radius, 2 ) );

              // current += beams[seg.id].energy * phase_f * rad * _int;
              current += beams[seg.id].energy * phase_f * rad;
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

bool HitCylinder( screen* screen, const vec4 start, const vec4 dir, const PhotonSeg& seg,
  CylIntersection& intersection ){
    bool intersects = false;
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
        if (th1<=0) return false;
        first_hit = origin_prime + ( dir_prime * th1 );
        intersects = true;

        if(y1<=1){
          second_hit = origin_prime + ( dir_prime * t1 );
        } else {
          float th2 = t1 + (t1-t0) * (y1-1) / (y0-y1);
          if (th2>0){
            second_hit = origin_prime + ( dir_prime * th2 );
          }
        }
      }
    }
    else if ( y0 >= 0 && y0 <= 1 )
    {
      // hit the cylinder bit
      if( t0 <= 0 ) return false;
      first_hit = origin_prime + ( dir_prime * t0 );
      intersects = true;

      if( y1 >= 0 ){
        if( y1 <= 1 ){
          second_hit = origin_prime + ( dir_prime * t1 );
        } else {
          float th2 = t1 + (t1-t0) * (y1-1) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
          }
        }
      } else {
        float th2 = t0 + (t1-t0) * (y0) / (y0-y1);
        if (th2>0){
          second_hit = origin_prime + ( dir_prime * th2 );
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
        intersects = true;
        if( y1 >= 0 ){
          second_hit = origin_prime + ( dir_prime * t1 );
        } else {
          float th2 = t0 + (t1-t0) * (y0) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
          }
        }
      }
    }
    if( ( t0 > 0 ) && ( t1 > 0 ) && ( t1 > t0 ) ){
      intersection.tb_minus      = first_hit.y / S_inv;
      intersection.tb_plus       = second_hit.y / S_inv;
      intersection.tc_minus      = t0 / S_inv;
      intersection.tc_plus       = t1 / S_inv;
    }
    return intersects;
}

bool ClosestIntersection(vec4 start, vec4 dir,
                         Intersection &closestIntersections) {
  bool found = false;
  closestIntersections.distance = m;

  mat4 matrix;  TransformationMatrix(matrix);

  for(int i = 0; i < triangles.size(); i++){
    vec4 v0 = matrix * triangles[i].v0;
    vec4 v1 = matrix * triangles[i].v1;
    vec4 v2 = matrix * triangles[i].v2;

    vec3 e1 = vec3(v1.x-v0.x, v1.y-v0.y, v1.z-v0.z);
    vec3 e2 = vec3(v2.x-v0.x, v2.y-v0.y, v2.z-v0.z);
    vec3 b = vec3(start.x-v0.x, start.y-v0.y, start.z-v0.z);

    vec3 direction;
    direction.x = dir.x;
    direction.y = dir.y;
    direction.z = dir.z;

    mat3 A(-direction,e1,e2);
    vec3 x_vec = glm::inverse( A ) * b;

    if(x_vec.x >= 0 && x_vec.y >= 0 && x_vec.z >= 0 && (x_vec.y+x_vec.z) < 1){
      if(x_vec.x < closestIntersections.distance){
        found = true;
        closestIntersections.position = start + dir * x_vec.x;
        closestIntersections.distance = x_vec.x;
        closestIntersections.index = i;
      }
    }
  }
  return found;
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
  ClosestIntersection( start, direction, c_i );

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
