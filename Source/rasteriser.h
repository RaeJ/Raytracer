#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include <stdint.h>
#include <random>

using namespace std;
using glm::vec3;
using glm::vec2;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using glm::ivec2;


#define SCREEN_WIDTH 512
#define SCREEN_HEIGHT 512
#define FULLSCREEN_MODE false
#define PI 3.14159265
#define SSAA 1
const float focal = SCREEN_WIDTH * SSAA;

vec3 purple( 0.75f, 0.15f, 0.75f );
vec3 white(  0.75f, 0.75f, 0.75f );
vec3 black(  0.0f, 0.0f, 0.0f );

struct Pixel
{
  int x;
  int y;
  vec4 normal;
  vec3 reflectance;
};

struct Vertex
{
  vec4 position;
  vec4 normal;
  vec3 reflectance;
};

struct AABB
{
  vec4 max;
  vec4 min;
  vec4 mid;
};

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void VertexShader( const Vertex& v, Pixel& p );
void ComputePolygonEdges( screen* screen, const vector<Vertex>& vertices );
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );
void DrawBoundingBox( screen* screen, AABB bound );
void DrawLine( screen* screen, Vertex& p1, Vertex& p2, vec3 colour );
void PixelShader( screen* screen, int x, int y, vec3 colour );

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

void DrawLine( screen* screen, Vertex& v1, Vertex& v2, vec3 colour ){
  Pixel proj1; VertexShader( v1, proj1 );
  Pixel proj2; VertexShader( v2, proj2 );

  int delta_x = abs( proj1.x - proj2.x );
  int delta_y = abs( proj1.y - proj2.y );
  int h = (int) sqrt( ( pow( delta_x, 2 ), pow( delta_y, 2 ) ) );
  int p_num = max( h, max( delta_x, delta_y ) ) + 1;

  vector<Pixel> result( p_num );

  Interpolate( proj1, proj2, result );

  for( int i=0; i<result.size(); i++ ){
    PixelShader( screen, result[i].x, result[i].y, colour );
  }
}

void ComputePolygonEdges( screen* screen, const vector<Vertex>& vertices ){
  int V = vertices.size();
  vector<Pixel> projectedVertices( V );

  for( int i=0; i<V; i++ ){
    VertexShader( vertices[i], projectedVertices[i] );
  }

  vector< vector<Pixel> > edges( V );
  for( int i=0; i<V; i++ ){
    int j = ( i + 1 ) % V;
    Pixel p_i1 = projectedVertices[i];
    Pixel p_i2 = projectedVertices[j];

    int delta_x = abs( p_i1.x - p_i2.x );
    int delta_y = abs( p_i1.y - p_i2.y );
    int h = (int) sqrt( ( pow( delta_x, 2 ), pow( delta_y, 2 ) ) );
    int p_num = max( h, max( delta_x, delta_y ) ) + 1;

    vector<Pixel> result( p_num );

    Interpolate( p_i1, p_i2, result );

    edges[i] = result;
  }

  for( int i=0; i<edges.size(); i++ ){
    vector<Pixel> edge = edges[i];
    for( int j=0; j<edge.size(); j++ ){
      PixelShader( screen, edge[j].x, edge[j].y, purple );
    }
  }
}

void VertexShader( const Vertex& vertex, Pixel& p ){
  int x = 0; int y = 0;

  // 4D position
  vec4 v = vertex.position;

  if( v.z != 0 ){
    // The 2D position
    x = (int) ( focal * ( v.x / (float) v.z ) ) + ( ( SCREEN_WIDTH * SSAA ) / (float) 2 );
    y = (int) ( focal * ( v.y / (float) v.z ) ) + ( ( SCREEN_HEIGHT * SSAA ) / (float) 2 );
  }

  p.x = x;  p.y = y;
  p.normal      = vertex.normal;
  p.reflectance = vertex.reflectance;
}

void PixelShader( screen* screen, int x, int y, vec3 colour )
{
  if( (x < SCREEN_WIDTH && x > 0) && (y < SCREEN_HEIGHT && y > 0)){
    PutPixelSDL( screen, x, y, colour );
  }
}

void Interpolate( Pixel a, Pixel b, vector<Pixel>& result ){
  float x, y;

  int N = result.size();
  vec3 colour = a.reflectance;
  vec4 normal = a.normal;


  if( N == 1 )
  {
    result[0].x           = (int) ( b.x + a.x ) / (float) 2;
    result[0].y           = (int) ( b.y + a.y ) / (float) 2;
    result[0].reflectance = colour;
    result[0].normal      = normal;
  }
  else
  {
    x     = ( b.x - a.x ) / ( float ) ( N - 1 );
    y     = ( b.y - a.y ) / ( float ) ( N - 1 );

    for(int i=0; i<result.size(); i++){
      int result_x = (a.x + (i*x));
      int result_y = (a.y + (i*y));

      if( result_y > max( a.y, b.y ) ){
        result_y = max( a.y, b.y );
      }
      if( result_y < min( a.y, b.y ) ){
        result_y = min( a.y, b.y );
      }

      result[i].x           = result_x;
      result[i].y           = result_y;
      result[i].reflectance = colour;
      result[i].normal      = normal;

    }
  }
}
