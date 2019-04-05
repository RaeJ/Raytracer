#ifndef CAST_PHOTONS_H
#define CAST_PHOTONS_H

#include <glm/glm.hpp>
#include "TestModelH.h"
#include "rasteriser.h"
#include "Tessendorf.h"
#include "Intersections.h"
#include "Control.h"

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
  vec3 energy;
  int index_hit;
  bool absorbed;
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

// -------------------------------------------------------------------------- //
// GLOBAL VARIABLES

vector<PhotonSeg> segments;

unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::mt19937 generator (seed);
std::uniform_real_distribution<double> uniform(0.0, 1.0);
// std::uniform_real_distribution<double> uniform_small(0.01, 0.1);
std::uniform_real_distribution<double> uniform_small(0.01, 0.04);
std::uniform_real_distribution<double> uniform_PI(0.0, PI);

// -------------------------------------------------------------------------- //
// FUNCTIONS

AABB CastPhotonBeams( int number,
                      vector<PhotonBeam>& beams,
                      const mat4& matrix,
                      const vector<Triangle>& triangles );
vec4 FindDirection( vec4 origin, vec4 centre, float radius );
void BuildTree( Node* parent, vector<PhotonSeg>& child );
void BoundPhotonBeams( vector<PhotonBeam>& beams, vector<PhotonSeg>& items, const vector<Triangle>& triangles );
void CastBeam( int bounce,
               vec3 energy,
               vec4 origin,
               vec4 direction,
               vec4& min_point,
               vec4& max_point,
               vector<PhotonBeam>& beams,
               float offset,
               float radius,
               const vector<Triangle>& triangles,
               const mat4& matrix,
               PhotonBeam& b
             );
bool intersectPlane( const vec3& n,
                    const vec3& p0,
                    const vec3& l0,
                    const vec3& l,
                    float& t ) ;
float Transmittance( float length, float ext );

// -------------------------------------------------------------------------- //
// DRAWING FUNCTIONS

void DrawBoundedBeams( screen* screen, vector<AABB> items );
void DrawBoundingBox( screen* screen, AABB bound );
void DrawBeam( screen* screen, PhotonBeam& b, vec3 colour );
void DrawTree( Node* parent, screen* screen );

// -------------------------------------------------------------------------- //
// IMPLEMENTATION

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
   left_child.mid = ( left_child.min + left_child.max ) / 2.0f;
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
   right_child.mid = ( right_child.min + right_child.max ) / 2.0f;
   Node *right_node = newNode( right_child );
   parent->right = right_node;
   BuildTree( parent->right, r );
 } else {
   parent -> right = NULL;
 }

}

// TODO: Fix this function at some point
void BoundPhotonBeams( vector<PhotonBeam>& beams, vector<PhotonSeg>& items, const vector<Triangle>& triangles ){
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

   float step   = 0.02;

   if( b.ada_width ){
     step = length;
   }


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

     beam_seg.min    = vec4( ( fmin( beam_seg.start.x, beam_seg.end.x) ),
                             ( fmin( beam_seg.start.y, beam_seg.end.y) ),
                             ( fmax( beam_seg.start.z, beam_seg.end.z) ), 1.0f );
     beam_seg.max    = vec4( ( fmax( beam_seg.start.x, beam_seg.end.x) ),
                             ( fmax( beam_seg.start.y, beam_seg.end.y) ),
                             ( fmin( beam_seg.start.z, beam_seg.end.z) ), 1.0f );

     if( length > 0.001 ){
       if( !( beam_seg.start.y == start.y ) ){
         beam_seg.min  = vec4( beam_seg.min.x - component.x,
                               beam_seg.min.y - component.y,
                               beam_seg.min.z + component.z, 1.0f );
       }
       if( !( ( vec3( beam_seg.end ) == vec3( end ) ) ) ){
         beam_seg.max  = vec4( beam_seg.max.x + component.x,
                               beam_seg.max.y + component.y,
                               beam_seg.max.z - component.z, 1.0f );
       } else if( b.index_hit != -1 ){
         Triangle tri = triangles[ b.index_hit ];
         vec4 tri_normal = abs( tri.normal );
         if( tri_normal.x > ( tri_normal.y || tri_normal.z ) ){
           beam_seg.max  = vec4( beam_seg.max.x,
                                 beam_seg.max.y + component.y,
                                 beam_seg.max.z - component.z, 1.0f );
         } else if ( tri_normal.y > ( tri_normal.x || tri_normal.z ) ){
           beam_seg.max  = vec4( beam_seg.max.x + component.x,
                                 beam_seg.max.y,
                                 beam_seg.max.z - component.z, 1.0f );
         } else if ( tri_normal.z > ( tri_normal.x || tri_normal.y ) ){
           beam_seg.max  = vec4( beam_seg.max.x + component.x,
                                 beam_seg.max.y + component.y,
                                 beam_seg.max.z, 1.0f );
         }
       }
     }

     beam_seg.mid    = ( beam_seg.min + beam_seg.max ) / 2.0f;

     items.push_back( beam_seg );
   }
 }
}

AABB CastPhotonBeams( int number, vector<PhotonBeam>& beams,
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
    float offset = uniform_small( generator ) * 0.1;
    float r      = uniform_small( generator );
    float r_small= uniform_small( generator );
    vec3 w_u     = glm::normalize( vec3( direction.x + r_small,
                                         direction.y,
                                         direction.z ) );
    vec3 w_v     = glm::normalize( vec3( direction.x,
                                         direction.y,
                                         direction.z - r_small ) );
    beam.omega_u = vec4( w_u, 1.0f );
    beam.omega_v = vec4( w_v, 1.0f );

    CastBeam( 0, energy, origin, direction, min_point, max_point,
              beams, offset, r, triangles, matrix, beam );
  }

  // for (double phi = 0.; phi < 2*PI; phi += PI/20.) // Azimuth [0, 2PI]
  //   {
  //       for (double theta = 0.; theta < PI; theta += PI/50.) // Elevation [0, PI]
  //       {
  //           float x        = cos(phi) * sin(theta);
  //           float y        = sin(phi) * sin(theta);
  //           float z        =            cos(theta);
  //           vec4 direction = glm::normalize( vec4( x, y, z, 1.0f ) );
  //           float offset   = 0.0f;
  //           CastBeam( 0, energy, origin, direction, min_point, max_point, beams,
  //                     offset, 0.02, triangles, matrix, beam );
  //       }
  //   }

  AABB root;
  root.min = min_point;
  root.max = max_point;
  root.mid = ( min_point + max_point ) / 2.0f;

  return root;
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

void CastBeam( int bounce, vec3 energy, vec4 origin, vec4 direction,
               vec4& min_point, vec4& max_point, vector<PhotonBeam>& beams,
               float offset, float radius, const vector<Triangle>& triangles,
               const mat4& matrix, PhotonBeam& beam ){

   Intersection hit;
   float diff;
   if( ClosestIntersection( origin, direction, hit, matrix, triangles ) ){
     // PhotonBeam beam;
     // TODO: Work out why the offset is not working as expected
     beam.offset    = offset;
     beam.radius    = radius;
     beam.energy    = energy;
     beam.index_hit = hit.index;

     beam.start     = origin + beam.offset;
     beam.end       = hit.position;

     float absorbed = uniform( generator );
     if( absorbed <= ABSORBED ){
       beam.absorbed= true;
     } else{
       beam.absorbed= false;
     }

     beams.push_back( beam );

     max_point.x = fmax( beam.end.x, fmax( beam.start.x, max_point.x ) );
     max_point.y = fmax( beam.end.y, fmax( beam.start.y, max_point.y ) );
     max_point.z = fmin( beam.end.z, fmin( beam.start.z, max_point.z ) );
     min_point.x = fmin( beam.end.x, fmin( beam.start.x, min_point.x ) );
     min_point.y = fmin( beam.end.y, fmin( beam.start.y, min_point.y ) );
     min_point.z = fmax( beam.end.z, fmax( beam.start.z, min_point.z ) );

     diff        = glm::length( vec3( hit.position - origin ) );

     if( ( bounce < BOUNCES ) && ( !beam.absorbed ) ){
       // float rand            =  dis( gen ) * 0.01;
       int num               = bounce + 1;
       Triangle hit_triangle = triangles[hit.index];
       vec4 normal           = hit_triangle.normal;
       vec4 incident         = hit.position + direction;
       // TODO: Should the below be diffuse instead of specular?
       vec3 refl             = glm::reflect( vec3( incident ), vec3( normal ) );
       vec4 reflected        = vec4( glm::normalize( refl ), 1.0f );
       float transmitted     = Transmittance( diff, extinction_c );
       vec3 new_energy       = energy * transmitted;
       PhotonBeam bounced;
       bounced.ada_width = false;
       CastBeam( num, new_energy, hit.position, reflected,
                 min_point, max_point, beams, 0.0f, radius,
                 triangles, matrix, bounced );
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
     beam.offset    = offset;
     beam.radius    = radius;
     beam.energy    = energy;
     beam.start     = origin + beam.offset;
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
     beam.end       = vec4( start + ( t_min * dir ), 1.0f );
     beams.push_back( beam );

     diff        = glm::length( vec3( beam.end - beam.start ) );

     max_point.x = fmax( beam.end.x, fmax( beam.start.x, max_point.x ) );
     max_point.y = fmax( beam.end.y, fmax( beam.start.y, max_point.y ) );
     max_point.z = fmin( beam.end.z, fmin( beam.start.z, max_point.z ) );
     min_point.x = fmin( beam.end.x, fmin( beam.start.x, min_point.x ) );
     min_point.y = fmin( beam.end.y, fmin( beam.start.y, min_point.y ) );
     min_point.z = fmax( beam.end.z, fmax( beam.start.z, min_point.z ) );
   }

   float scattered  = uniform( generator );
   if( scattered <= ( scattering_c / extinction_c ) ){
     float t_s         = diff * uniform( generator );
     vec4 direc        = vec4( glm::normalize( vec3( direction ) ), 0 );
     vec4 start        = origin + ( t_s * direc );
     float the         = 2 * PI * uniform( generator );
     float phi         = acos(1 - 2 * uniform( generator ) );
     float x           = sin( phi ) * cos( the );
     float y           = sin( phi ) * sin( the );
     float z           = cos( phi );
     vec3 dir          = glm::normalize( vec3( x, y, z ) );
     vec4 dir_sample   = vec4( dir.x, dir.y, dir.z, 1.0f );
     float transmitted = Transmittance( t_s, extinction_c );
     vec3 new_energy   = energy * transmitted;

     PhotonBeam scattered;
     scattered.ada_width = false;
     CastBeam( bounce, new_energy, start, dir_sample,
               min_point, max_point, beams, 0.0f,
               radius, triangles, matrix, scattered );
   }
}

vec4 FindDirection( vec4 origin, vec4 centre, float radius ){
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

float Transmittance( float length, float ext ){
  float power     = - ext * length;
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
