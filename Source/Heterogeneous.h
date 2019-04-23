#ifndef HETEROGENEOUS_H
#define HETEROGENEOUS_H

#include "Tessendorf.h"
#include "rasteriser.h"

void DrawBox( screen* screen, vec4 min, vec4 max, vec3 colour );
void ConvertTo2D( const vec3& point, vec2& p );
float Extinction( const vec4 start,
                  const vec4 end,
                  const Grid grid,
                  vector<vec2>& dist_ext );
float Extinction3D( const vec4 start,
                    const vec4 dir,
                    const Grid grid,
                    vector<vec2>& dist_ext );
float FindAverageExtinction( const vec2& start, const vec2& direction,
                            const vector<float>& distances,
                            const vector<ivec2>& indexes,
                            const vec2& cell_dimension,
                            vector<float>& extinctions );
float FindAverageExtinction3D( const vec3& start, const vec3& direction,
                             const vector<float>& distances,
                             const vector<vec3>& indexes,
                             const vec3& cell_dimension,
                             vector<float>& extinctions );

// TODO: Test extinction 3D
float Extinction3D( const vec4 start,
                    const vec4 end,
                    const Grid grid,
                    vector<vec2>& dist_ext ){

    float t_x, t_y, t_z, t_max;
    vec3 grid_min = vec3( -1, -1, 2 );
    vec3 ray_direction = glm::normalize( vec3( end ) - vec3( start ) ); // assumed normalized
    vec3 grid_resolution = vec3( grid.side_points - 1, grid.side_points - 1, ACTUAL_WIDTH );
    vec3 cell_dimension = vec3( ACTUAL_WIDTH, ACTUAL_WIDTH, ACTUAL_WIDTH ) / grid_resolution;
    vec3 delta_t;
    vec3 ray_orig_grid = vec3( start ) - grid_min;

    t_max = glm::length( vec3( end ) - vec3( start ) );

    if ( ray_direction.x < 0 ) {
        delta_t.x = -cell_dimension.x / ray_direction.x;
        t_x = ( floor( ray_orig_grid.x / cell_dimension.x ) * cell_dimension.x
            - ray_orig_grid.x ) / ray_direction.x;
    }
    else {
        delta_t.x = cell_dimension.x / ray_direction.x;
        t_x = ( ( floor( ray_orig_grid.x / cell_dimension.x ) + 1 ) * cell_dimension.x
            - ray_orig_grid.x ) / ray_direction.x;
    }
    if ( ray_direction.y < 0 ) {
        delta_t.y = -cell_dimension.y / ray_direction.y;
        t_y = ( floor( ray_orig_grid.y / cell_dimension.y ) * cell_dimension.y
            - ray_orig_grid.y ) / ray_direction.y;
    }
    else {
        delta_t.y = cell_dimension.y / ray_direction.y;
        t_y = ( ( floor( ray_orig_grid.y / cell_dimension.y ) + 1 ) * cell_dimension.y
            - ray_orig_grid.y ) / ray_direction.y;
    }
    if ( ray_direction.z < 0 ) {
        delta_t.z = -cell_dimension.z / ray_direction.z;
        t_z = ( floor( ray_orig_grid.z / cell_dimension.z ) * cell_dimension.z
            - ray_orig_grid.z ) / ray_direction.z;
    }
    else {
        delta_t.z = cell_dimension.z / ray_direction.z;
        t_z = ( ( floor( ray_orig_grid.z / cell_dimension.z ) + 1 ) * cell_dimension.z
            - ray_orig_grid.z ) / ray_direction.z;
    }

    float t = 0;
    vec3 cell_index = floor( ray_orig_grid / cell_dimension );

    vector<vec3> indexes;
    vector<float> distances;
    while ( true ) {
      indexes.push_back( cell_index );
      distances.push_back( t );
      // cout << "( " << cell_index.x << ", " << cell_index.y << ", " << cell_index.z << " )" << endl;
        if ( ( t_x < t_y ) && ( t_x < t_z )) {
            t = t_x; // current t, next intersection with cell along ray
            t_x += delta_t.x; // increment, next crossing along x
            if ( ray_direction.x < 0 )
                cell_index.x -= 1;
            else
                cell_index.x += 1;
        } else if( t_y < t_z ){
            t = t_y;
            t_y += delta_t.y; // increment, next crossing along y
            if ( ray_direction.y < 0 )
                cell_index.y -= 1;
            else
                cell_index.y += 1;
        } else {
            t = t_z;
            t_z += delta_t.z; // increment, next crossing along z
            if ( ray_direction.z < 0 )
                cell_index.z -= 1;
            else
                cell_index.z += 1;
        }
        // if some condition is met break from the loop
        if ( cell_index.x < 0 || cell_index.y < 0 ||
            cell_index.x > grid_resolution.x ||
            cell_index.y > grid_resolution.y ||
            t > t_max )
            break;
    }
    vector<float> extinctions;
    float average_extinction = FindAverageExtinction3D( ray_orig_grid,
                                    ray_direction,
                                    distances,
                                    indexes,
                                    cell_dimension,
                                    extinctions );

    for( int i=0; i<indexes.size(); i++ ){
      dist_ext.push_back( vec2( distances[i], extinctions[i] ) );
    }
    return average_extinction;
}

float FindAverageExtinction3D( const vec3& start, const vec3& direction,
                             const vector<float>& distances,
                             const vector<vec3>& indexes,
                             const vec3& cell_dimension,
                             vector<float>& extinctions ){
  float average_extinction = 0;
  for( int i=0; i<indexes.size(); i++ ){
    vec3 intersection_point = start + ( direction * distances[i] );
    vec3 min = cell_dimension * vec3( indexes[i] );
    vec3 delta = intersection_point - min;
    vec3 proportion = abs( delta ) / cell_dimension; //One of these will normally be zero

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
    if( ext_i  < 0 ){
      cout << "proportion x: " << proportion.x << endl;
      cout << "proportion y: " << proportion.y << endl;
      cout << "left: " << left << endl;
      cout << "right: " << right << endl;
      cout << "tl_ext: " << tl_ext << endl;
      cout << "bl_ext: " << bl_ext << endl;
      ext_i = 0;
    }
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

float Extinction( const vec4 start,
                  const vec4 dir,
                  const Grid grid,
                  vector<vec2>& dist_ext ){
    dist_ext.clear();

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
            cellIndex.x > gridResolution.x ||
            cellIndex.y > gridResolution.y )
            break;
    }
    vector<float> extinctions;
    float average_extinction = FindAverageExtinction( rayOrigin,
                                    rayDirection,
                                    distances,
                                    indexes,
                                    cellDimension,
                                    extinctions );
    for( int i=0; i<indexes.size(); i++ ){
      dist_ext.push_back( vec2( distances[i], extinctions[i] ) );
    }
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

void DrawBox( screen* screen, vec4 min, vec4 max, vec3 colour ){
  vector<Vertex> vertices(4);

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

  DrawLine( screen, vertices[0], vertices_1[0], colour );
  DrawLine( screen, vertices[1], vertices_1[1], colour );
  DrawLine( screen, vertices[2], vertices_1[2], colour );
  DrawLine( screen, vertices[3], vertices_1[3], colour );

  DrawLine( screen, vertices[0], vertices[1], colour );
  DrawLine( screen, vertices[1], vertices[2], colour );
  DrawLine( screen, vertices[2], vertices[3], colour );
  DrawLine( screen, vertices[3], vertices[0], colour );

  DrawLine( screen, vertices_1[0], vertices_1[1], colour );
  DrawLine( screen, vertices_1[1], vertices_1[2], colour );
  DrawLine( screen, vertices_1[2], vertices_1[3], colour );
  DrawLine( screen, vertices_1[3], vertices_1[0], colour );

  Pixel proj1; Vertex min_v; min_v.position = min;
  Pixel proj2; Vertex max_v; max_v.position = max;

  VertexShader( min_v, proj1 );
  VertexShader( max_v, proj2 );
  PixelShader( screen, proj1.x, proj1.y, white );
  PixelShader( screen, proj2.x, proj2.y, black );
}

#endif
