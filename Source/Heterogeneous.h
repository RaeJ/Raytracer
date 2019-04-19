#ifndef HETEROGENEOUS_H
#define HETEROGENEOUS_H

#include "Tessendorf.h"
#include "rasteriser.h"

void ConvertTo2D( const vec3& point, vec2& p );
float Extinction( screen* screen,
                 const vec4 start,
                 const vec4 dir,
                 const Grid grid );
float FindAverageExtinction( const vec2& start, const vec2& direction,
                            const vector<float>& distances,
                            const vector<ivec2>& indexes,
                            const vec2& cell_dimension,
                            vector<float>& extinctions );

float Extinction( screen* screen,
                 const vec4 start,
                 const vec4 dir,
                 const Grid grid /* Grid should already be matrix oriented*/){

    vec2 rayOrigin, projected_point;
    float t_x, t_y;
    ConvertTo2D( vec3( start ), rayOrigin );
    ConvertTo2D( vec3( start + dir ), projected_point );

    vec2 rayDirection = glm::normalize( projected_point - rayOrigin ); // assumed normalized
    vec2 gridResolution = vec2( grid.side_points - 1, grid.side_points - 1 );
    vec2 cellDimension = vec2( SCREEN_WIDTH, SCREEN_HEIGHT ) / gridResolution;
    vec2 deltaT, nextCrossingT;
    vec2 rayOrigGrid = rayOrigin;// - vec2( -1, -1 ); //gridMin;

    if (rayDirection.x < 0) {
        deltaT.x = -cellDimension.x / rayDirection.x;
        t_x = (floor(rayOrigGrid.x / cellDimension.x) * cellDimension.x
            - rayOrigGrid.x) / rayDirection.x;
    }
    else {
        deltaT.x = cellDimension.x / rayDirection.x;
        t_x = ((floor(rayOrigGrid.x / cellDimension.x) + 1) * cellDimension.x
            - rayOrigGrid.x) / rayDirection.x;
    }
    if (rayDirection.y < 0) {
        deltaT.y = -cellDimension.y / rayDirection.y;
        t_y = (floor(rayOrigGrid.y / cellDimension.y) * cellDimension.y
            - rayOrigGrid.y) / rayDirection.y;
    }
    else {
        deltaT.y = cellDimension.y / rayDirection.y;
        t_y = ((floor(rayOrigGrid.y / cellDimension.y) + 1) * cellDimension.y
            - rayOrigGrid.y) / rayDirection.y;
    }

    float t = 0;
    ivec2 cellIndex = floor( rayOrigin / cellDimension ); // origin of the ray (cell index)

    vector<ivec2> indexes;
    vector<float> distances;
    while ( true ) {
      indexes.push_back( cellIndex );
      distances.push_back( t );
        if (t_x < t_y) {
            t = t_x; // current t, next intersection with cell along ray
            t_x += deltaT.x; // increment, next crossing along x
            if (rayDirection.x < 0)
                cellIndex.x -= 1;
            else
                cellIndex.x += 1;
        }
        else {
            t = t_y;
            t_y += deltaT.y; // increment, next crossing along y
            if (rayDirection.y < 0)
                cellIndex.y -= 1;
            else
                cellIndex.y += 1;
        }
        // if some condition is met break from the loop
        if ( cellIndex.x < 0 || cellIndex.y < 0 ||
            cellIndex.x > gridResolution.x || cellIndex.y > gridResolution.y )
            break;
    }
    vector<float> extinctions;
    float average_extinction = FindAverageExtinction( rayOrigin,
                                    rayDirection,
                                    distances,
                                    indexes,
                                    cellDimension,
                                    extinctions );
    return average_extinction;
}

float FindAverageExtinction( const vec2& start, const vec2& direction,
                             const vector<float>& distances,
                             const vector<ivec2>& indexes,
                             const vec2& cell_dimension,
                             vector<float>& extinctions ){
  float average_extinction = 0;
  for( int i=0; i<indexes.size(); i++ ){
    vec2 intersection_point = start + ( direction * distances[i] );
    vec2 min = cell_dimension * vec2( indexes[i] );
    vec2 delta = intersection_point - min;
    vec2 proportion = delta / cell_dimension; //One of these will normally be zero

    int t_l = ( ( indexes[i].y ) * ( GRID.side_points ) ) + indexes[i].x;
    int t_r = t_l + 1;
    int b_l = ( ( indexes[i].y + 1 ) * ( GRID.side_points ) ) + indexes[i].x;
    int b_r = b_l + 1;

    float tl_ext = GRID.geometric_points[t_l].z;
    float bl_ext = GRID.geometric_points[b_l].z;
    float tr_ext = GRID.geometric_points[t_r].z;
    float br_ext = GRID.geometric_points[b_r].z;
    float left   = tl_ext + ( ( bl_ext - tl_ext ) * proportion.y );
    float right  = tr_ext + ( ( br_ext - tr_ext ) * proportion.y );

    float ext_i  = ( left + right ) * proportion.x;
    extinctions.push_back( ext_i );

    average_extinction += ext_i;
  }
  return ( average_extinction / ( indexes.size() ) );
}

void ConvertTo2D( const vec3& point, vec2& p ){
  float x = 0; float y = 0;

  if( point.z != 0 ){
    // The 2D position
    x = ( focal * ( point.x / (float) point.z ) ) + ( ( SCREEN_WIDTH * SSAA ) / (float) 2 );
    y = ( focal * ( point.y / (float) point.z ) ) + ( ( SCREEN_HEIGHT * SSAA ) / (float) 2 );
  }

  p.x = x;  p.y = y;
}
#endif
