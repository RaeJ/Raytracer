#ifndef CAST_PHOTONS_H
#define CAST_PHOTONS_H

#include <glm/glm.hpp>
#include "Control.h"
#include "TestModelH.h"
#include "Intersections.h"
#include "Heterogeneous.h"


// -------------------------------------------------------------------------- //
// STRUCTS

struct PhotonBeam
{
  vec4 start;
  vec4 end;
  vec4 omega_u;
  vec4 omega_v;
  bool ada_width;
  float offset;
  float radius;
  glm::dvec3 energy;
  int index_hit;
  bool absorbed;
  bool scattered;
};

struct NodeGen
{
    int key;
    AABB aabb;
    bool leaf;
    PhotonSeg segment;
    vector<NodeGen *>child;
};

 // Utility function to create a new tree node
NodeGen *newNodeGen(AABB data)
{
    NodeGen *temp = new NodeGen;
    temp->aabb = data;
    temp->leaf = false;
    PhotonSeg init; init.id = -1;
    temp->segment = init;
    return temp;
}

struct Node
{
  AABB aabb;
  PhotonSeg segments[2];
  bool leaf;
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
  node->leaf = false;

  // Initialize left and right children as NULL
  node->left = NULL;
  node->right = NULL;
  return(node);
}

// -------------------------------------------------------------------------- //
// GLOBAL VARIABLES

int tree_segments = 0;

vector<PhotonSeg> segments;
vector<PhotonSeg> lost_segments;

std::mt19937 generator ( SEED );
std::uniform_real_distribution<double> uniform(0.0, 1.0);
std::uniform_real_distribution<double> uniform_offset(-0.01, 0.01);
std::uniform_real_distribution<double> uniform_radius(0.0, 0.05);
std::uniform_real_distribution<double> uniform_beam(0.02f, 0.03f);
std::uniform_real_distribution<double> uniform_small(0.02f, 0.06f);
std::uniform_real_distribution<double> uniform_PI(0.0, PI);
std::normal_distribution<double> normal(0.0, 0.25);


// -------------------------------------------------------------------------- //
// FUNCTIONS

AABB CastPhotonBeams( int number,
                      vector<PhotonBeam>& beams_const,
                      vector<PhotonBeam>& beams_scattered,
                      const mat4& matrix,
                      const vector<Triangle>& triangles );
vec4 FindDirection( vec4 origin, vec4 centre, float radius );
void BuildTree( Node* parent, vector<PhotonSeg>& child, int leftover );
void BuildKdTree( NodeGen* parent, vector<PhotonSeg>& child, vec3& whd, vec3& center, int leftover );
void BoundPhotonBeams( vector<PhotonBeam>& beams, vector<PhotonSeg>& items, const vector<Triangle>& triangles );
void CastBeam( int bounce,
               vec3 energy,
               vec4 origin,
               vec4 direction,
               vec4& min_point,
               vec4& max_point,
               vector<PhotonBeam>& beams_const,
               vector<PhotonBeam>& beams_scattered,
               float radius,
               const vector<Triangle>& triangles,
               const mat4& matrix,
               PhotonBeam& b
             );
AABB RecastPhotonBeams( vector<PhotonBeam>& beams_const,
                        vector<PhotonBeam>& beams_scattered,
                        const mat4& matrix,
                        const vector<Triangle>& triangles );

bool intersectPlane( const vec3& n,
                    const vec3& p0,
                    const vec3& l0,
                    const vec3& l,
                    float& t ) ;
double Transmittance( double length, double ext );

// -------------------------------------------------------------------------- //
// DRAWING FUNCTIONS

void DrawBoundedBeams( screen* screen, vector<AABB> items );
void DrawBoundingBox( screen* screen, AABB bound );
void DrawBeam( screen* screen, PhotonBeam& b, vec3 colour );
void DrawTree( Node* parent, screen* screen );

// -------------------------------------------------------------------------- //
// IMPLEMENTATION

void BoundPhotonBeams( vector<PhotonBeam>& beams, vector<PhotonSeg>& items, const vector<Triangle>& triangles ){
 vec3 x_dir     = glm::normalize( vec3( 1, 0, 0 ) );
 vec3 y_dir     = glm::normalize( vec3( 0, 1, 0 ) );
 vec3 z_dir     = glm::normalize( vec3( 0, 0, 1 ) );

 for( int i=0; i<beams.size(); i++ ){
   PhotonBeam b = beams[i];

   vec4 start   = b.start;
   vec4 end     = b.end;

   vec3 diff    = vec3( end ) - vec3( start );
   vec3 dir     = glm::normalize( diff );
   float length = glm::length( diff );
   float j=0;

   float theta_x = fmin( acos( glm::dot( dir, x_dir ) ), acos( glm::dot( dir, -x_dir ) ) );
   float theta_y = fmin( acos( glm::dot( dir, y_dir ) ), acos( glm::dot( dir, -y_dir ) ) );
   float theta_z = fmin( acos( glm::dot( dir, z_dir ) ), acos( glm::dot( dir, -z_dir ) ) );

   float step   = length; //0.2;

   vector<vec2> dist_ext;
   int current_step = 1;
   float mean = extinction_c;
   if( HETEROGENEOUS ){
     Extinction3D( start, end, GRID, dist_ext );
     // step = dist_ext[current_step].x; //[0] would be zero
     mean = dist_ext[current_step-1].y;
   } else if( b.ada_width ) {
     step = length;
   }

   vec3 prior   = vec3( start.x, start.y, start.z );

   while( j<length && length>1e-6 ){
     if( HETEROGENEOUS ){
       step           = dist_ext[current_step].x - dist_ext[current_step-1].x;
     }
     PhotonSeg beam_seg;
     vec3 next = prior + ( dir * step );

     beam_seg.start  = vec4( prior.x, prior.y, prior.z, 1.0f );
     if( HETEROGENEOUS ){
       mean = mean + ( ( dist_ext[current_step].y - mean ) / ( current_step + 1 ) );
     } else {
       mean = extinction_c;
     }

     beam_seg.c_ext  = mean;
     beam_seg.seg_id = current_step;

     if( length <= glm::length( vec3( next ) - vec3( start ) ) ){
       j = glm::length( length );
       beam_seg.end = vec4( end.x, end.y, end.z, 1.0f );
     } else {
       j = glm::length( next - vec3( start ) );
       beam_seg.end = vec4( next.x, next.y, next.z, 1.0f );
     }

     prior = vec3( beam_seg.end );

     beam_seg.orig_start = start;
     beam_seg.orig_start.w = 1.0f;

     if( b.ada_width ){
       vec3 omega_u        = glm::normalize( vec3( b.omega_u ) );
       float cos_theta     = glm::dot( dir, omega_u );
       float hyp_length    = j / cos_theta;
       vec4 end_u          = start + ( b.omega_u * hyp_length );
       beam_seg.radius     = glm::length( vec3( beam_seg.end ) - vec3( end_u ) );
       beam_seg.ada_width  = true;
     } else {
       beam_seg.radius = b.radius;
       beam_seg.ada_width  = false;
     }

     if( b.scattered ){
       beam_seg.scattered = true;
     } else {
       beam_seg.scattered = false;
     }

     beam_seg.id     = i;

     beam_seg.min    = vec4( ( fmin( beam_seg.start.x, beam_seg.end.x) ),
                             ( fmin( beam_seg.start.y, beam_seg.end.y) ),
                             ( fmax( beam_seg.start.z, beam_seg.end.z) ), 1.0f );
     beam_seg.max    = vec4( ( fmax( beam_seg.start.x, beam_seg.end.x) ),
                             ( fmax( beam_seg.start.y, beam_seg.end.y) ),
                             ( fmin( beam_seg.start.z, beam_seg.end.z) ), 1.0f );

     if( length > 0.001 ){
       float hypotenuse = 0;
       float adj = 0;
       if( theta_x < theta_y && theta_x < theta_z ){
         hypotenuse = beam_seg.radius / sin( theta_x );
         adj = beam_seg.radius / cos( theta_x );
         float height = adj * sin( theta_x );
         beam_seg.min  = vec4( beam_seg.min.x - height,
                               beam_seg.min.y - hypotenuse,
                               beam_seg.min.z + hypotenuse, 1.0f );
         beam_seg.max  = vec4( beam_seg.max.x + height,
                               beam_seg.max.y + hypotenuse,
                               beam_seg.max.z - hypotenuse, 1.0f );
       } else if( theta_y < theta_z ){
         hypotenuse = beam_seg.radius / sin( theta_y );
         adj = beam_seg.radius / cos( theta_y );
         float height = adj * sin( theta_y );
         beam_seg.min  = vec4( beam_seg.min.x - hypotenuse,
                               beam_seg.min.y - height,
                               beam_seg.min.z + hypotenuse, 1.0f );
         beam_seg.max  = vec4( beam_seg.max.x + hypotenuse,
                               beam_seg.max.y + height,
                               beam_seg.max.z - hypotenuse, 1.0f );
       } else {
         hypotenuse = beam_seg.radius / sin( theta_z );
         adj = beam_seg.radius / cos( theta_z );
         float height = adj * sin( theta_z );
         beam_seg.min  = vec4( beam_seg.min.x - hypotenuse,
                               beam_seg.min.y - hypotenuse,
                               beam_seg.min.z + height, 1.0f );
         beam_seg.max  = vec4( beam_seg.max.x + hypotenuse,
                               beam_seg.max.y + hypotenuse,
                               beam_seg.max.z - height, 1.0f );
       }

     }

     beam_seg.mid    = ( beam_seg.min + beam_seg.max ) / 2.0f;
     if( HETEROGENEOUS ){
       beam_seg.s_ext = dist_ext[current_step-1].y;

       beam_seg.e_ext = dist_ext[current_step].y;

       if( current_step == dist_ext.size()-1 ){
         j = length + 1;
       }
       current_step++;
     }

     items.push_back( beam_seg );
   }
 }
}

AABB RecastPhotonBeams( vector<PhotonBeam>& beams_const,
                        vector<PhotonBeam>& beams_scattered,
                        const mat4& matrix,
                        const vector<Triangle>& triangles ){
  vec4 min_point = vec4( m, m, -m, 1 );
  vec4 max_point = vec4( -m, -m, m, 1 );

  for( int i=0; i<beams_const.size(); i++ ){
    PhotonBeam beam = beams_const[i];
    vec4 direction = vec4( glm::normalize( vec3( beam.end ) - vec3( beam.start ) ), 1.0f );
    CastBeam( 0, beam.energy, beam.start, direction, min_point, max_point,
              beams_const, beams_scattered, beam.radius, triangles, matrix, beam );
  }

  AABB root;
  root.min = min_point;
  root.max = max_point;
  root.mid = ( min_point + max_point ) / 2.0f;

  return root;
}

AABB CastPhotonBeams( int number, vector<PhotonBeam>& beams_const,
                      vector<PhotonBeam>& beams_scattered,
                      const mat4& matrix, const vector<Triangle>& triangles ){

  vec4 min_point = vec4( m, m, -m, 1 );
  vec4 max_point = vec4( -m, -m, m, 1 );

  vec4 origin    = matrix * light_position;
  // vec4 centre    = vec4( origin.x, origin.y + 0.5, origin.z, 1.0f );
  vec4 centre    = vec4( origin.x, origin.y + 1.5, origin.z, 1.0f );
  // TODO: make this radius value useful
  float radius   = 0.05f;

  vec3 energy    = light_power;

  PhotonBeam beam;

  for( int i=0; i<number; i++ ){
    vec4 direction = FindDirection( origin, centre, radius );
    direction = glm::normalize( direction );
    beam.ada_width = ADAPTIVE;
    float r      = uniform_beam( generator );
    float r_small= uniform_small( generator );
    vec3 w_u     = glm::normalize( vec3( direction.x + r_small,
                                         direction.y,
                                         direction.z ) );
    vec3 w_v     = glm::normalize( vec3( direction.x,
                                         direction.y,
                                         direction.z - r_small ) );
    if( FIXED_RADIUS ){
      r          = RADIUS;
      w_u        = glm::normalize( vec3( direction.x + r,
                                         direction.y,
                                         direction.z ) );
      w_v        = glm::normalize( vec3( direction.x,
                                         direction.y,
                                         direction.z - r ) );
    }

    beam.omega_u = vec4( w_u, 1.0f );
    beam.omega_v = vec4( w_v, 1.0f );

    float t        = 2* PI * uniform_radius( generator );
    float u        = uniform_radius( generator ) + uniform_radius( generator );
    float w_width;
    if( u > 1 ) w_width = 2 - u; else w_width = u;
    float x_offset = w_width*cos(t);
    float y_offset = uniform_offset( generator ) * 0.1;
    float z_offset = w_width*sin(t);
    vec4 displace  = vec4( x_offset, y_offset, z_offset, 0 );

    vec4 start     = origin + displace;

    CastBeam( 0, energy, start, direction, min_point, max_point,
              beams_const, beams_scattered, r, triangles, matrix, beam );

  }

  // for (double phi = 0.; phi < 2*PI; phi += PI/13.) // Azimuth [0, 2PI]
  //   {
  //       for (double theta = 0.; theta < PI; theta += PI/13.) // Elevation [0, PI]
  //       {
  //           float x        = cos(phi) * sin(theta);
  //           float y        = sin(phi) * sin(theta);
  //           float z        =            cos(theta);
  //           vec4 direction = glm::normalize( vec4( x, y, z, 1.0f ) );
  //           beam.ada_width = ADAPTIVE;
  //           CastBeam( 0, energy, origin, direction, min_point, max_point,
  //                     beams, RADIUS, triangles, matrix, beam );
  //       }
  //   }

  AABB root;
  root.min = min_point;
  root.max = max_point;
  root.mid = ( min_point + max_point ) / 2.0f;

  return root;
}

void CastBeam( int bounce, vec3 energy, vec4 origin, vec4 direction,
               vec4& min_point, vec4& max_point,
               vector<PhotonBeam>& add_to, vector<PhotonBeam>& other,
               float radius, const vector<Triangle>& triangles,
               const mat4& matrix, PhotonBeam& beam ){
   bool scatt = false;
   beam.scattered = false;

   Intersection hit;
   float t_s, average_extinction;
   vector<vec2> dist_ext;
   if( ClosestIntersection( origin, direction, hit, matrix, triangles ) ){
     t_s = glm::length( vec3( hit.position ) - vec3( origin ) );
     if( t_s < 0.0005 ){
       return;
     }
   } else {
     t_s = ACTUAL_WIDTH;
   }
   if( HETEROGENEOUS ){
     average_extinction = Extinction3D( origin, origin + ( direction * t_s ), GRID, dist_ext );
   } else {
     average_extinction = extinction_c;
   }


   float diff;
   float scattered  = uniform( generator );
   if( ( scattered <= ( scattering_c / extinction_c ) ) && SCATTER ){
     // TODO: check distance is correct
     float tmp_t_s     = -( log( 1 - uniform( generator ) ) / average_extinction );
     if( ( tmp_t_s < t_s ) && ( tmp_t_s > 0 ) ){
       t_s               = tmp_t_s;
       vec4 start        = origin + ( t_s * glm::normalize( direction ) );
       start.w           = 1;
       float the         = 2 * PI * uniform( generator );
       float phi         = acos(1 - 2 * uniform( generator ) );
       float x           = sin( phi ) * cos( the );
       float y           = sin( phi ) * sin( the );
       float z           = cos( phi );
       vec3 dir          = glm::normalize( vec3( x, y, z ) );
       vec4 dir_sample   = vec4( dir.x, dir.y, dir.z, 1.0f );
       float transmitted = 0;
       if( HETEROGENEOUS ){

         float mean = 0;
         float current_step = 0;
         while( dist_ext[current_step].x < t_s ){
           mean = mean + ( ( dist_ext[current_step].y - mean ) / ( current_step + 1 ) );
           current_step++;
         }
         transmitted = Transmittance( t_s, mean );
       } else {
         transmitted = Transmittance( t_s, average_extinction );
       }
       vec3 new_energy   = energy * transmitted;
       if( new_energy == vec3( 0, 0, 0 ) ){
         beam.absorbed = true;
       } else {
         new_energy.x = fmin( energy.x, fmax( new_energy.x, 0 ) );
         new_energy.y = fmin( energy.y, fmax( new_energy.y, 0 ) );
         new_energy.z = fmin( energy.z, fmax( new_energy.z, 0 ) );

         PhotonBeam scattered;
         scattered.scattered = true;
         scattered.ada_width = false;
         CastBeam( bounce, new_energy, start, dir_sample,
                   min_point, max_point, other, add_to,
                   radius, triangles, matrix, scattered );
       }

       if( SHORT_BEAMS ){
         scatt = true;
       }
     }
   } else{
     // is absorbed
     float tmp_t_s     = -( log( 1 - uniform( generator ) ) / average_extinction );
     if( ( tmp_t_s < t_s ) && ( tmp_t_s > 0 ) ){
       t_s            = tmp_t_s;
       beam.radius    = radius;
       beam.energy    = energy;
       beam.index_hit = -1;
       beam.start     = origin;
       beam.end       = origin + ( t_s * glm::normalize( direction ) );
       beam.absorbed  = false;
       return;
     }
   }

   if( ClosestIntersection( origin, direction, hit, matrix, triangles ) ){
     // PhotonBeam beam;
     // TODO: Work out why the offset is not working as expected
     // beam.offset    = offset;
     beam.radius    = radius;
     beam.energy    = energy;
     beam.index_hit = hit.index;

     beam.start     = origin;// + beam.offset;
     if( scatt ){
       beam.end     = origin + ( t_s * direction );
       beam.end.w   = 1.0f;
     } else {
       beam.end     = hit.position;
     }

     float absorbed = uniform( generator );
     if( absorbed < ABSORBED ){
       beam.absorbed= true;
     } else{
       beam.absorbed= false;
     }

     add_to.push_back( beam );

     max_point.x = fmax( beam.end.x, fmax( beam.start.x, max_point.x ) );
     max_point.y = fmax( beam.end.y, fmax( beam.start.y, max_point.y ) );
     max_point.z = fmin( beam.end.z, fmin( beam.start.z, max_point.z ) );
     min_point.x = fmin( beam.end.x, fmin( beam.start.x, min_point.x ) );
     min_point.y = fmin( beam.end.y, fmin( beam.start.y, min_point.y ) );
     min_point.z = fmax( beam.end.z, fmax( beam.start.z, min_point.z ) );

     diff        = glm::length( vec3( hit.position - origin ) );

     if( ( bounce < BOUNCES ) && ( !beam.absorbed ) && ( !scatt ) ){
       // float rand            =  dis( gen ) * 0.01;
       int num               = bounce + 1;
       Triangle hit_triangle = triangles[hit.index];
       vec4 normal           = hit_triangle.normal;
       vec4 incident         = hit.position + direction;
       // TODO: Should the below be diffuse instead of specular?
       vec3 refl             = glm::reflect( vec3( incident ), vec3( normal ) );
       vec4 reflected        = vec4( glm::normalize( refl ), 1.0f );
       vec4 bounce_dir       = FindDirection( hit.position, hit.position + ( reflected * 0.5f ), 0 );
       vec4 new_direction    = vec4( glm::normalize( vec3( bounce_dir ) ), 1.0f );
       if( scattered > 0.8 ) new_direction = reflected;
       float transmitted = 0;
       if( HETEROGENEOUS ){
         float mean = 0;
         float current_step = 0;
         while( dist_ext[current_step].x < t_s ){
           mean = mean + ( ( dist_ext[current_step].y - mean ) / ( current_step + 1 ) );
           current_step++;
         }
         transmitted = Transmittance( t_s, mean );
       } else {
         transmitted = Transmittance( t_s, average_extinction );
       }
       vec3 new_energy       = energy * transmitted;
       if( new_energy == vec3( 0, 0, 0 ) ){
         beam.absorbed = true;
       } else {
         new_energy.x = fmin( energy.x, fmax( new_energy.x, 0 ) );
         new_energy.y = fmin( energy.y, fmax( new_energy.y, 0 ) );
         new_energy.z = fmin( energy.z, fmax( new_energy.z, 0 ) );

         PhotonBeam bounced;
         bounced.ada_width = false;
         CastBeam( num, new_energy, hit.position + ( normal * 0.0001f ), new_direction,
                   min_point, max_point, other, add_to, radius,
                   triangles, matrix, bounced );
       }
     } else if( ABSORBED > 0 ) {
       beam.absorbed= true;
     }
   } else {
     vec4 top_left  = vec4( -1, -1, -1, 1 );
     vec4 top_right = vec4( 1, -1, -1, 1 );
     vec4 bot_left  = vec4( -1, 1, -1, 1 );
     vec4 bot_right = vec4( 1, 1, -1, 1 );
     vec4 converge  = matrix * vec4( 0, 0, camera.z, 1.0f );
     vec4 top       = vec4( matrix * ( ( top_left + top_right ) / 2.0f ) );
     vec4 right     = vec4( matrix * ( ( top_right + bot_right ) / 2.0f ) );
     vec4 bot       = vec4( matrix * ( ( bot_right + bot_left ) / 2.0f ) );
     vec4 left      = vec4( matrix * ( ( bot_left + top_left ) / 2.0f ) );
     vec3 p_0_0     = vec3( ( top + converge ) / 2.0f );
     vec3 p_0_1     = vec3( ( right + converge ) / 2.0f );
     vec3 p_0_2     = vec3( ( bot + converge ) / 2.0f );
     vec3 p_0_3     = vec3( ( left + converge ) / 2.0f );
     vec3 n_0_0     = glm::normalize( glm::cross( p_0_0 - vec3( top ),
                                      vec3( ( matrix * top_left ) - top ) ) );
     vec3 n_0_1     = glm::normalize( glm::cross( p_0_1 - vec3( right ),
                                      vec3( ( matrix * top_right ) - right ) ) );
     vec3 n_0_2     = glm::normalize( glm::cross( p_0_2 - vec3( bot ),
                                      vec3( ( matrix * bot_right ) - bot ) ) );
     vec3 n_0_3     = glm::normalize( glm::cross( p_0_3 - vec3( left ),
                                      vec3( ( matrix * bot_left ) - left ) ) );
     vec3 dir       = glm::normalize( vec3( direction ) );
     vec3 start     = vec3( origin );

     float t;
     // PhotonBeam beam;
     beam.absorbed  = false;
     // beam.offset    = offset;
     beam.radius    = radius;
     beam.energy    = energy;
     beam.start     = origin;// + beam.offset;
     beam.index_hit = -1;

     float t_min = m;
     if( intersectPlane( n_0_0, p_0_0, start, dir, t ) ){
       // Intersects with top
       if( t < t_min ) t_min = t;
     }
     if( intersectPlane( n_0_1, p_0_1, start, dir, t ) ){
       // Intersects with right
       if( t < t_min ) t_min = t;
     }
     if( intersectPlane( n_0_2, p_0_2, start, dir, t ) ){
       // Intersects with bottom
       if( t < t_min ) t_min = t;
     }
     if( intersectPlane( n_0_3, p_0_3, start, dir, t ) ){
       // Intersects with left
       if( t < t_min ) t_min = t;
     }
     if( t_min == m ){
       cout << "No intersection found" << endl;
       return;
     }

     if( scatt ){
       beam.end     = origin + ( t_s * direction );
       beam.end.w   = 1.0f;
     } else {
       beam.end       = vec4( start + ( t_min * dir ), 1.0f );
     }

     add_to.push_back( beam );

     diff        = glm::length( vec3( beam.end - beam.start ) );

     max_point.x = fmax( beam.end.x, fmax( beam.start.x, max_point.x ) );
     max_point.y = fmax( beam.end.y, fmax( beam.start.y, max_point.y ) );
     max_point.z = fmin( beam.end.z, fmin( beam.start.z, max_point.z ) );
     min_point.x = fmin( beam.end.x, fmin( beam.start.x, min_point.x ) );
     min_point.y = fmin( beam.end.y, fmin( beam.start.y, min_point.y ) );
     min_point.z = fmax( beam.end.z, fmax( beam.start.z, min_point.z ) );
   }


}

bool intersectPlane(const vec3 &n, const vec3 &p0, const vec3 &l0, const vec3 &l, float &t)
{
    // assuming vectors are all normalized
    float denom = glm::dot( n, l );
    // Note, needed to make use of the abs() function
    if ( abs( denom ) > 1e-6 ) {
        vec3 p0l0 = p0 - l0;
        t = glm::dot(p0l0, n) / denom;
        return ( t >= 0 );
    }

    return false;
}

void BuildKdTree( NodeGen* parent, vector<PhotonSeg>& child, vec3& whd, vec3& center, int leftover ){
  if( child.size() <= 0 ){
    return;
  }
  if( child.size() == 1 ){
    PhotonSeg box = child[0];
    parent->segment = box;
    parent->leaf    = true;
    return;
  }
  parent->leaf = false;

  vector<PhotonSeg> l;
  vector<PhotonSeg> r;
  vector<PhotonSeg> leftovers;
  vec3 new_whd    = whd;

  bool same = true;
  if( ( whd.x > whd.y ) && ( whd.x > whd.z ) ){
    new_whd.x = whd.x / 2.0f;
    for( int i=0; i<child.size(); i++ ){
      PhotonSeg box = child[i];
      if( abs( box.min.x - box.max.x ) >= new_whd.x ){
        AABB aabb; aabb.min = box.min; aabb.max = box.max;
        NodeGen *new_node = newNodeGen( aabb );
        new_node->leaf = true;
        new_node->segment = box;
        (parent->child).push_back( new_node );
        tree_segments++;
      } else {
        if( box.mid.x < center.x ){
          l.push_back( box );
          same = false;
        }  else if( box.mid.x > center.x ) {
          r.push_back( box );
          same = false;
        } else if( !same ){
          leftovers.push_back( box );
        }
      }
    }
    if( leftovers.size() > ( ( l.size() + r.size() ) / 2 ) ) same = true;
  } else if( whd.y > whd.z ){
    new_whd.y = whd.y / 2.0f;
    for( int i=0; i<child.size(); i++ ){
      PhotonSeg box = child[i];
      if( abs( box.min.y - box.max.y ) >= new_whd.y ){
        AABB aabb; aabb.min = box.min; aabb.max = box.max;
        NodeGen *new_node = newNodeGen( aabb );
        new_node->leaf = true;
        new_node->segment = box;
        (parent->child).push_back( new_node );
        tree_segments++;
      } else {
        if( box.mid.y < center.y ){
          l.push_back( box );
          same = false;
        }  else if( box.mid.y > center.y ){
          r.push_back( box );
          same = false;
        } else if( !same ){
           leftovers.push_back( box );
        }
      }
    }
    if( leftovers.size() > ( ( l.size() + r.size() ) / 2 ) ) same = true;
  } else {
    new_whd.z = whd.z / 2.0f;
    for( int i=0; i<child.size(); i++ ){
      PhotonSeg box = child[i];
      if( abs( box.min.z - box.max.z ) >= new_whd.z ){
        AABB aabb; aabb.min = box.min; aabb.max = box.max;
        NodeGen *new_node = newNodeGen( aabb );
        new_node->leaf = true;
        new_node->segment = box;
        (parent->child).push_back( new_node );
        tree_segments++;
      } else {
        if( box.mid.z > center.z ){
          l.push_back( box );
          same = false;
        } else if( box.mid.z < center.z ){
          r.push_back( box );
          same = false;
        } else if( !same ){
          leftovers.push_back( box );
        }
      }
    }
    if( leftovers.size() > ( ( l.size() + r.size() ) / 2 ) ) same = true;
  }

  // if( leftover != leftovers.size() ){
    for( int i = 0; i<leftovers.size(); i++ ){
      if( uniform( generator ) < 0.5 ){
        l.push_back( leftovers[i] );
      } else {
        r.push_back( leftovers[i] );
      }
    }
  // }
    // cout << "5." << endl;
   vec3 diff_whd = abs( whd - new_whd );
   vec3 new_center;
   new_center.x = center.x - ( diff_whd.x / 2.0f );
   new_center.y = center.y - ( diff_whd.y / 2.0f );
   new_center.z = center.z + ( diff_whd.z / 2.0f );
   vec4 min = vec4( m, m, -m, 1 ); vec4 max = vec4( -m, -m, m, 1 );
   int l_size = l.size();
   if( l_size != 0 ){
     max.x = new_center.x + ( new_whd.x / 2.0f );
     max.y = new_center.y + ( new_whd.y / 2.0f );
     max.z = new_center.z - ( new_whd.z / 2.0f );
     min.x = new_center.x - ( new_whd.x / 2.0f );
     min.y = new_center.y - ( new_whd.y / 2.0f );
     min.z = new_center.z + ( new_whd.z / 2.0f );

     AABB left_child;
     left_child.min = vec4( min.x, min.y, min.z, 1.0f );
     left_child.max = vec4( max.x, max.y, max.z, 1.0f );
     left_child.mid = ( left_child.min + left_child.max ) / 2.0f;
     NodeGen *left_node = newNodeGen( left_child );
     (parent->child).push_back(left_node);
     BuildKdTree( left_node, l, new_whd, new_center, leftovers.size() );
   }
   // cout << "6." << endl;

   new_center.x = center.x + ( diff_whd.x / 2.0f );
   new_center.y = center.y + ( diff_whd.y / 2.0f );
   new_center.z = center.z - ( diff_whd.z / 2.0f );
   min = vec4( m, m, -m, 1 ); max = vec4( -m, -m, m, 1 );
   int r_size = r.size();
   if( r_size != 0 ){
     max.x = new_center.x + ( new_whd.x / 2.0f );
     max.y = new_center.y + ( new_whd.y / 2.0f );
     max.z = new_center.z - ( new_whd.z / 2.0f );
     min.x = new_center.x - ( new_whd.x / 2.0f );
     min.y = new_center.y - ( new_whd.y / 2.0f );
     min.z = new_center.z + ( new_whd.z / 2.0f );

     AABB right_child;
     right_child.min = vec4( min.x, min.y, min.z, 1.0f );
     right_child.max = vec4( max.x, max.y, max.z, 1.0f );
     right_child.mid = ( right_child.min + right_child.max ) / 2.0f;
     NodeGen *right_node = newNodeGen( right_child );
     (parent->child).push_back(right_node);
     BuildKdTree( right_node, r, new_whd, new_center, leftovers.size() );
   }

}

void BuildTree( Node* parent, vector<PhotonSeg>& child, int leftover ){
  // cout << "1." << endl;
 if( child.size() <= 2 ) {
   parent->leaf = true;
   for( int i=0; i<child.size(); i++ ){
     // parent->segments.push_back( child[i] );
     parent->leaf = true;
     parent->segments[i] = child[i];
     segments.push_back( child[i] );
     tree_segments++;
   }
   return;
 }
 parent->leaf = false;

 vec4 diff     = abs( parent->aabb.max - parent->aabb.min );
 vec4 mid      = parent->aabb.mid;

 vector<PhotonSeg> l;
 vector<PhotonSeg> r;
 vector<PhotonSeg> leftovers;
 // cout << "2." << endl;

 bool same = true;
 int dim   = -1;
 if( ( diff.x > diff.y ) && ( diff.x > diff.z ) ){
   for( int i=0; i<child.size(); i++ ){
     PhotonSeg box = child[i];
     if( box.mid.x < mid.x ){
       l.push_back( box );
       same = false;
     }  else if( box.mid.x > mid.x ) {
       r.push_back( box );
       same = false;
     } else if( !same ){
       leftovers.push_back( box );
     }
   }
   if( leftovers.size() > ( ( l.size() + r.size() ) / 2 ) ) same = true;
   if( same ) dim = 0;
 } else if( diff.y > diff.z ){
   for( int i=0; i<child.size(); i++ ){
     PhotonSeg box = child[i];
     if( box.mid.y < mid.y ){
       l.push_back( box );
       same = false;
     }  else if( box.mid.y > mid.y ){
       r.push_back( box );
       same = false;
     } else if( !same ){
        leftovers.push_back( box );
     }
   }
   if( leftovers.size() > ( ( l.size() + r.size() ) / 2 ) ) same = true;
   if( same ) dim = 1;
 } else {
   for( int i=0; i<child.size(); i++ ){
     PhotonSeg box = child[i];
     if( box.mid.z > mid.z ){
       l.push_back( box );
       same = false;
     } else if( box.mid.z < mid.z ){
       r.push_back( box );
       same = false;
     } else if( !same ){
       leftovers.push_back( box );
     }
   }
   if( leftovers.size() > ( ( l.size() + r.size() ) / 2 ) ) same = true;
   if( same ) dim = 2;
 }
 // cout << "3." << endl;
 // TODO: There has to be a better way of doing this
 if( same ){
   l.clear();
   r.clear();
   if( dim == 0 ){
     if( diff.y > diff.z ){
       for( int i=0; i<child.size(); i++ ){
         PhotonSeg box = child[i];
         if( box.mid.y < mid.y ){
           l.push_back( box );
           same = false;
         }  else if( box.mid.y > mid.y ){
           r.push_back( box );
           same = false;
         } else if( !same ){
           leftovers.push_back( box );
         }
       }
       if( leftovers.size() > ( ( l.size() + r.size() ) / 2 ) ) same = true;
       if( same ){
         l.clear();
         r.clear();
         for( int i=0; i<child.size(); i++ ){
           PhotonSeg box = child[i];
           if( box.mid.z < mid.z ){
             l.push_back( box );
             same = false;
           }  else if( box.mid.z > mid.z ){
             r.push_back( box );
             same = false;
           } else {
             leftovers.push_back( box );
           }
         }
       }
     } else {
       for( int i=0; i<child.size(); i++ ){
         PhotonSeg box = child[i];
         if( box.mid.z > mid.z ){
           l.push_back( box );
           same = false;
         } else if( box.mid.z < mid.z ){
           r.push_back( box );
           same = false;
         } else if( !same ){
           leftovers.push_back( box );
         }
       }
       if( leftovers.size() > ( ( l.size() + r.size() ) / 2 ) ) same = true;
       if( same ){
         l.clear();
         r.clear();
         for( int i=0; i<child.size(); i++ ){
           PhotonSeg box = child[i];
           if( box.mid.y < mid.y ){
             l.push_back( box );
             same = false;
           }  else if( box.mid.y > mid.y ){
             r.push_back( box );
             same = false;
           } else {
             leftovers.push_back( box );
           }
         }
       }
     }
   } else if( dim == 1 ){
     if( diff.x > diff.z ){
       for( int i=0; i<child.size(); i++ ){
         PhotonSeg box = child[i];
         if( box.mid.x < mid.x ){
           l.push_back( box );
           same = false;
         }  else if( box.mid.x > mid.x ) {
           r.push_back( box );
           same = false;
         } else if( !same ){
           leftovers.push_back( box );
         }
       }
       if( leftovers.size() > ( ( l.size() + r.size() ) / 2 ) ) same = true;
       if( same ){
         l.clear();
         r.clear();
         for( int i=0; i<child.size(); i++ ){
           PhotonSeg box = child[i];
           if( box.mid.z < mid.z ){
             l.push_back( box );
             same = false;
           }  else if( box.mid.z > mid.z ){
             r.push_back( box );
             same = false;
           } else {
             leftovers.push_back( box );
           }
         }
       }
     } else {
       for( int i=0; i<child.size(); i++ ){
         PhotonSeg box = child[i];
         if( box.mid.z > mid.z ){
           l.push_back( box );
           same = false;
         } else if( box.mid.z < mid.z ){
           r.push_back( box );
           same = false;
         } else if( !same ){
           leftovers.push_back( box );
         }
       }
       if( leftovers.size() > ( ( l.size() + r.size() ) / 2 ) ) same = true;
       if( same ){
         l.clear();
         r.clear();
         for( int i=0; i<child.size(); i++ ){
           PhotonSeg box = child[i];
           if( box.mid.x < mid.x ){
             l.push_back( box );
             same = false;
           }  else if( box.mid.x > mid.x ){
             r.push_back( box );
             same = false;
           } else {
             leftovers.push_back( box );
           }
         }
       }
     }
   } else if( dim == 2 ){
     if( diff.x > diff.y ){
       for( int i=0; i<child.size(); i++ ){
         PhotonSeg box = child[i];
         if( box.mid.x < mid.x ){
           l.push_back( box );
           same = false;
         }  else if( box.mid.x > mid.x ) {
           r.push_back( box );
           same = false;
         } else if( !same ){
           leftovers.push_back( box );
         }
       }
       if( leftovers.size() > ( ( l.size() + r.size() ) / 2 ) ) same = true;
       if( same ){
         l.clear();
         r.clear();
         for( int i=0; i<child.size(); i++ ){
           PhotonSeg box = child[i];
           if( box.mid.y < mid.y ){
             l.push_back( box );
             same = false;
           }  else if( box.mid.y > mid.y ){
             r.push_back( box );
             same = false;
           } else {
             leftovers.push_back( box );
           }
         }
       }
     } else {
       for( int i=0; i<child.size(); i++ ){
         PhotonSeg box = child[i];
         if( box.mid.y < mid.y ){
           l.push_back( box );
           same = false;
         }  else if( box.mid.y > mid.y ){
           r.push_back( box );
           same = false;
         } else if( !same ){
           leftovers.push_back( box );
         }
       }
       if( leftovers.size() > ( ( l.size() + r.size() ) / 2 ) ) same = true;
       if( same ){
         l.clear();
         r.clear();
         for( int i=0; i<child.size(); i++ ){
           PhotonSeg box = child[i];
           if( box.mid.x < mid.x ){
             l.push_back( box );
             same = false;
           }  else if( box.mid.x > mid.x ){
             r.push_back( box );
             same = false;
           } else {
             leftovers.push_back( box );
           }
         }
       }
     }
   }
 }
// cout << "4." << endl;
if( !HETEROGENEOUS || ( leftover > leftovers.size() )  ){
  for( int i = 0; i<leftovers.size(); i++ ){
    if( uniform( generator ) < 0.5 ){
      l.push_back( leftovers[i] );
    } else {
      r.push_back( leftovers[i] );
    }
  }
} else {
  for( int i = 0; i<leftovers.size(); i++ ){
    lost_segments.push_back( leftovers[i] );
  }
}
  // cout << "5." << endl;
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
   left_child.mid = ( left_child.min + left_child.max ) / 2.0f;
   Node *left_node = newNode( left_child );
   parent->left = left_node;
   BuildTree( parent->left, l, leftovers.size() );
 } else {
   parent -> left = NULL;
 }
 // cout << "6." << endl;

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
   right_child.mid = ( right_child.min + right_child.max ) / 2.0f;
   Node *right_node = newNode( right_child );
   parent->right = right_node;
   BuildTree( parent->right, r, leftovers.size() );
 } else {
   parent -> right = NULL;
 }
 // cout << "7." << endl;

}

vec4 FindDirection( vec4 origin, vec4 centre, float radius ){
  // float r1  = normal( generator );
  // float r2  = normal( generator );
  float r1  = uniform( generator );
  float r2  = uniform( generator );
  float the = 2 * PI * r1;
  float phi = sqrt( r2 );
  float x = centre.x + ( phi * cos( the ) );
  float z = centre.z + ( phi * sin( the ) );
  float y = centre.y + sqrt( 1 - r2 );
  vec4 position( x, y, z, 1.0f );

  return( position - origin );
}

double Transmittance( double length, double ext ){
  double power     = - ext * length;
  return exp( power );
}

// -------------------------------------------------------------------------- //
// DRAWING FUNCTIONS

void DrawBoundingBox( screen* screen, AABB bound ){
  vector<Vertex> vertices(4);
  vec4 max  = bound.max;
  vec4 min  = bound.min;

  vertices[0].position  = max;
  vertices[1].position  = vec4( min.x, max.y, max.z, 1.0f );
  vertices[2].position  = vec4( min.x, min.y, max.z, 1.0f );
  vertices[3].position  = vec4( max.x, min.y, max.z, 1.0f );

  ComputePolygonEdges( screen, vertices );

  vector<Vertex> vertices_1(4);
  vertices_1[0].position  = vec4( max.x, max.y, min.z, 1.0f );
  vertices_1[1].position  = vec4( min.x, max.y, min.z, 1.0f );
  vertices_1[2].position  = min;
  vertices_1[3].position  = vec4( max.x, min.y, min.z, 1.0f );

  ComputePolygonEdges( screen, vertices_1 );

  DrawLine( screen, vertices[0], vertices_1[0], purple );
  DrawLine( screen, vertices[1], vertices_1[1], purple );
  DrawLine( screen, vertices[2], vertices_1[2], purple );
  DrawLine( screen, vertices[3], vertices_1[3], purple );

  DrawLine( screen, vertices[0], vertices[1], purple );
  DrawLine( screen, vertices[1], vertices[2], purple );
  DrawLine( screen, vertices[2], vertices[3], purple );
  DrawLine( screen, vertices[3], vertices[0], purple );

  DrawLine( screen, vertices_1[0], vertices_1[1], purple );
  DrawLine( screen, vertices_1[1], vertices_1[2], purple );
  DrawLine( screen, vertices_1[2], vertices_1[3], purple );
  DrawLine( screen, vertices_1[3], vertices_1[0], purple );

  Pixel proj1; Vertex min_v; min_v.position = min;
  Pixel proj2; Vertex max_v; max_v.position = max;

  VertexShader( min_v, proj1 );
  VertexShader( max_v, proj2 );
  PixelShader( screen, proj1.x, proj1.y, white );
  PixelShader( screen, proj2.x, proj2.y, black );
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


#endif
