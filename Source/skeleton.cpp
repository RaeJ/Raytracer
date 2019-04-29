#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "BeamRadiance.h"
#include "Analysis.h"
#include <sstream>
#include <iostream>
#include <chrono>
#include <ctime>

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
           vector<PhotonBeam> beams_const,
           vector<PhotonBeam> beams_scatt,
           vector<PhotonSeg>& items );

void DrawStandard( screen* screen,
          vector<PhotonBeam> beams_const,
          vector<PhotonBeam> beams_scatt );

void TransformationMatrix( glm::mat4& m );
bool UserInput();
vec3 DirectLight( const Intersection& i );

void ProduceStopMotion();
void SingleRun();
void HeterogeneousRun();
void DrawHet( screen* screen, vector<PhotonBeam> beams_const, vector<PhotonBeam> beams_scattered, NodeGen* );
void RecurseTree( screen* screen, Node* parent_node, vec3 colour );
void RecurseGenTree( screen* screen, NodeGen* parent_node, vec3 colour );
void StandardRun();

// ------------------------------------------------------------------------- //

void RecurseTree( screen* screen, Node* parent_node, vec3 colour ){
  AABB box = parent_node->aabb;
  DrawBoundingBox(screen, box);
  PhotonSeg segments[2];
  segments[0] = parent_node->segments[0];
  segments[1] = parent_node->segments[1];
  for( int i = 0; i < 2; i++ ){
    // PhotonSeg seg = segments[i];
    // if( seg.id != -1 ){
    //   DrawBox( screen, seg.min, seg.max, colour );
    // }
  }
  if( parent_node -> left != NULL ){
    RecurseTree( screen, parent_node -> left, colour );
  }
  if( parent_node -> right != NULL ){
    RecurseTree( screen, parent_node -> right, colour );
  }
}

int main( int argc, char* argv[] )
{
  srand (time(NULL));
  if( RUN_ANALYSIS ){
    RunAnalysis();
  }

  LoadTestModel( triangles );

  if( HETEROGENEOUS ){
    HeterogeneousRun();
  } else {
    StandardRun();
    // SingleRun();
  }
  // ProduceStopMotion();

  return 0;
}

void RecurseGenTree( screen* screen, NodeGen* parent_node, vec3 colour ){
  AABB box = parent_node->aabb;
  PhotonSeg seg = parent_node->segment;
  DrawBoundingBox(screen, box);
  if( seg.id != -1 ){
    DrawBox( screen, seg.min, seg.max, vec3(1,1,1) );
  }
  for( int i=0; i<(parent_node->child).size(); i++ ){
    RecurseGenTree(screen,parent_node->child[i],colour);
  }
}

void HeterogeneousRun(){
  cout << "Creating grid" << endl;
  CreateSurface( 27, -3.0, 0.016 );
  mat4 matrix;  TransformationMatrix( matrix );
  root_matrix = matrix;

  float minimum = 10;
  float maximum = -10;
  for( int i=0; i<GRID.geometric_points.size(); i++ ){
    GRID.geometric_points[i] = matrix*GRID.geometric_points[i];
    if( GRID.geometric_points[i].z < minimum )
      minimum = GRID.geometric_points[i].z;
    if( GRID.geometric_points[i].z > maximum )
      maximum = GRID.geometric_points[i].z;
  }

  cout << "Minimum value: " << minimum << endl;
  cout << "Maximum value: " << maximum << endl;

  vector<PhotonBeam> beams_const;
  vector<PhotonBeam> beams_scattered;
  vector<PhotonSeg> items;

  cout << "Casting photons" << endl;
  root_aabb = CastPhotonBeams( PHOTON_NUMBER, beams_const, beams_scattered, matrix, triangles );
  BoundPhotonBeams( beams_const, items, triangles );
  BoundPhotonBeams( beams_scattered, items, triangles );
  cout << "Beams size: " << ( beams_const.size() + beams_scattered.size() ) << "\n";
  cout << "Segment size: " << items.size() << "\n";
  root = newNode( root_aabb );

  AABB aabb;
  aabb.min = vec4( -1, -1, 2, 1.0f );
  aabb.max = vec4( 1, 1, 4, 1.0f );
  NodeGen *new_root = newNodeGen( aabb );
  cout << "Building tree" << endl;

  // vec3 whd = vec3(ACTUAL_WIDTH,ACTUAL_WIDTH,ACTUAL_WIDTH);
  // vec3 center = vec3(0,0,3);
  // BuildKdTree( new_root, items, whd, center, 0 );

  BuildTree( root, items, 0 );

  cout << "Tree segment size: " << tree_segments << "\n";
  cout << "Calculating radiance" << endl;

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  // RecurseGenTree( screen, new_root, vec3(0,0,1));
  vec3 cell_dimension = vec3( ACTUAL_WIDTH, ACTUAL_WIDTH, ACTUAL_WIDTH ) /
                      vec3( GRID.side_points - 1, GRID.side_points - 1, ACTUAL_WIDTH );

  while( Update() )
    {
      DrawHet( screen, beams_const, beams_scattered, new_root );
      // for( int i=0; i< beams_const.size(); i++ ){
      //   DrawBeam(screen, beams_const[i], vec3(1,1,1));
      // }
      // for( int x=0; x<GRID.side_points - 1; x++ ){
      //      for( int y=0; y<GRID.side_points - 1; y++ ){
      //        vec4 min = vec4( ( vec3( x, y, 0) * cell_dimension ) + vec3( -1, -1, 2 ), 1.0f );
      //        vec4 max = vec4( ( x + 1 ) * cell_dimension.x - 1,
      //                         ( y + 1 ) * cell_dimension.y - 1,
      //                         ( 1 ) * cell_dimension.z + 2, 1.0f );
      //        DrawBox( screen, min, max, vec3( 0, 0, 0.5 ) );
      //      }
      //    }
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );
  KillSDL(screen);
}

void StandardRun(){
  mat4 matrix;  TransformationMatrix( matrix );
  vector<PhotonBeam> beams_const;
  vector<PhotonBeam> beams_scatt;
  vector<PhotonSeg> items;

  cout << "Casting photons" << endl;
  root_aabb = CastPhotonBeams( PHOTON_NUMBER, beams_const, beams_scatt, matrix, triangles );
  BoundPhotonBeams( beams_const, items, triangles );
  BoundPhotonBeams( beams_scatt, items, triangles );
  cout << "Beams size: " << ( beams_const.size() + beams_scatt.size() ) << "\n";
  cout << "Segment size: " << items.size() << "\n";
  root = newNode( root_aabb );
  cout << "Building tree" << endl;
  BuildTree( root, items, 0 );
  cout << "Tree segment size: " << tree_segments << "\n";
  cout << "Calculating radiance" << endl;

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  while( Update() )
    {
      DrawStandard( screen, beams_const, beams_scatt );
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );
  KillSDL(screen);
}

void SingleRun(){
  mat4 matrix;  TransformationMatrix( matrix );
  vector<PhotonBeam> beams_const;
  vector<PhotonBeam> beams_scattered;
  vector<PhotonSeg> items;

  cout << "Casting photons" << endl;
  root_aabb = CastPhotonBeams( PHOTON_NUMBER, beams_const, beams_scattered, matrix, triangles );
  BoundPhotonBeams( beams_const, items, triangles );
  BoundPhotonBeams( beams_scattered, items, triangles );
  cout << "Beams size: " << ( beams_const.size() + beams_scattered.size() ) << "\n";
  cout << "Segment size: " << items.size() << "\n";
  root = newNode( root_aabb );
  cout << "Building tree" << endl;
  BuildTree( root, items, 0 );
  cout << "Tree segment size: " << tree_segments << "\n";
  cout << "Calculating radiance" << endl;

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  while( Update() )
    {
      Draw( screen, beams_const, beams_scattered, items );
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );
  KillSDL(screen);
}

void ProduceStopMotion(){
  // auto start = std::chrono::system_clock::now();
  vector<PhotonBeam> beams_const;
  vector<PhotonBeam> beams_scattered;
  vector<PhotonSeg> items;

  CreateSurface( 27, -3.0, 0.016 );
  mat4 matrix;  TransformationMatrix( matrix );
  root_matrix = matrix;
  for( int i=0; i<GRID.geometric_points.size(); i++ ){
    GRID.geometric_points[i] = matrix*GRID.geometric_points[i];
  }

  root_aabb = CastPhotonBeams( PHOTON_NUMBER, beams_const, beams_scattered, matrix, triangles );
  BoundPhotonBeams( beams_const, items, triangles );
  BoundPhotonBeams( beams_scattered, items, triangles );
  root = newNode( root_aabb );
  BuildTree( root, items, 0 );

  int begin  = 16;
  int finish = 24;
  for( int i = begin; i<finish; i++ ){
    cout << ".";
  }
  cout << endl;

  for( int i=begin; i<finish; i++ ){
    screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

    Draw( screen, beams_const, beams_scattered, items );
    SDL_Renderframe(screen);

    // auto end = std::chrono::system_clock::now();
    // std::chrono::duration<double> elapsed_seconds = end-start;

    std::ostringstream oss;
    oss << "screenshot_" << i << "_.bmp";
    std::string filename = oss.str();

    char *file = new char[filename.length() + 1];
    std::strcpy(file, filename.c_str());

    SDL_SaveImage( screen, file );
    KillSDL(screen);

    cout << ".";

    beams_scattered.clear();
    items.clear();

    CreateSurface( 27, -3.0, i/1000 );
    root_matrix = matrix;
    for( int i=0; i<GRID.geometric_points.size(); i++ ){
      GRID.geometric_points[i] = matrix*GRID.geometric_points[i];
    }

    vector<PhotonBeam> beams_copy = beams_const;
    root_aabb = RecastPhotonBeams( beams_copy, beams_scattered, matrix, triangles );
    BoundPhotonBeams( beams_const, items, triangles );
    BoundPhotonBeams( beams_scattered, items, triangles );
    root = newNode( root_aabb );
    BuildTree( root, items, 0 );
  }
}

void DrawStandard( screen* screen,
          vector<PhotonBeam> beams_const,
          vector<PhotonBeam> beams_scatt ){
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

            BeamRadianceStandard( screen, start, direction,
                                c_i, root, current, beams_const,
                                beams_scatt, x );
          }
        }
      }
      // PutPixelSDL( screen, x / SSAA, y / SSAA, colour / (float) SSAA );
      PutPixelSDL( screen, x / SSAA, y / SSAA, current / (double) SSAA );
    }
    SDL_Renderframe(screen);
  }
}

void DrawHet( screen* screen,
              vector<PhotonBeam> beams_const,
              vector<PhotonBeam> beams_scattered, NodeGen* root_node )
{
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

            // BeamRadianceHet( screen,
            //                  start,
            //                  direction,
            //                  c_i,
            //                  root_node,
            //                  current,
            //                  beams_const,
            //                  beams_scattered, x );
            BVHBeamRadianceHet( screen, start, direction,
                                c_i, root, current, beams_const,
                                beams_scattered, x );
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

void Draw( screen* screen, vector<PhotonBeam> beams_const, vector<PhotonBeam> beams_scatt, vector<PhotonSeg>& items )
{
  // mat4 inverse_matrix = glm::inverse( root_matrix );
  mat4 matrix;

  TransformationMatrix( matrix );

  for( int x = 0; x < (SCREEN_WIDTH * SSAA); x+=SSAA ) {
    for( int y = 0; y < (SCREEN_HEIGHT * SSAA); y+=SSAA ) {
      glm::dvec3 current = vec3( 0, 0, 0 );
      vec3 colour = vec3( 0, 0, 0 );
      // cout << "Row x: " << x << endl; // 296
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

            BeamRadiance( screen, start, direction, c_i, root, current, beams_const, beams_scatt, x );
          }
          // Testing( screen, start, direction, colour, current, beams, items, matrix );
        }
      }
      // PutPixelSDL( screen, x / SSAA, y / SSAA, colour / (float) SSAA );
      PutPixelSDL( screen, x / SSAA, y / SSAA, current / (double) SSAA );
    }

    // vec3 cell_dimension = vec3( ACTUAL_WIDTH, ACTUAL_WIDTH, ACTUAL_WIDTH ) /
    //                       vec3( GRID.side_points - 1, GRID.side_points - 1, ACTUAL_WIDTH );
    // for( int x=0; x<GRID.side_points - 1; x++ ){
    //      for( int y=0; y<GRID.side_points - 1; y++ ){
    //        vec4 min = vec4( ( vec3( x, y, 0) * cell_dimension ) + vec3( -1, -1, 2 ), 1.0f );
    //        vec4 max = vec4( ( x + 1 ) * cell_dimension.x - 1,
    //                         ( y + 1 ) * cell_dimension.y - 1,
    //                         ( 1 ) * cell_dimension.z + 2, 1.0f );
    //        DrawBox( screen, min, max, vec3( 0, 0, 0.4 ) );
    //      }
    //    }
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
