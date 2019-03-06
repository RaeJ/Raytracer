#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include "rasteriser.h"

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

/* ----------------------------------------------------------------------------*/
/* STRUCTS                                                                     */

struct PhotonBeam
{
  vec4 start;
  vec4 end;
  float offset;
  float radius;
};

struct Node
{
  AABB aabb;
  struct Node *left;
  struct Node *right;
};

struct Node* newNode( AABB data )
{
  // Allocate memory for new node
  struct Node* node = (struct Node*)malloc(sizeof(struct Node));

  // Assign data to this node
  node->aabb = data;

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
vector<PhotonBeam> beams;

float m = std::numeric_limits<float>::max();

vec4 camera(0, 0, -3, 1.0);
vec3 theta( 0.0, 0.0, 0.0 );
// vec4 light_position(0,-1.2,-0.5,1);
vec4 light_position(0,-0.5,-0.7,1);
vec3 light_power = 14.f * vec3( 1, 1, 1 );
vec3 indirect_light = 0.5f*vec3( 1, 1, 1 );

std::random_device rd;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
std::normal_distribution<> dis(0.0, 1 );

AABB root;

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
bool ClosestIntersection(
  vec4 start,
  vec4 dir,
  Intersection & closestIntersections
);
void TransformationMatrix( glm::mat4& m );
void UserInput();
vec3 DirectLight( const Intersection& i );
AABB CastPhotonBeams( int number );
vec4 FindDirection( vec4 origin, vec4 centre, float radius );
void DrawBeam( screen* screen, PhotonBeam& b );
void DrawBoundedBeams( screen* screen );


int main( int argc, char* argv[] )
{
  LoadTestModel( triangles );
  root = CastPhotonBeams( 2 );

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  while( NoQuitMessageSDL() )
    {
      Update();
      Draw(screen);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

void Draw( screen* screen )
{
  mat4 matrix;  TransformationMatrix(matrix);
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  for( int x = 0; x < SCREEN_WIDTH; x++ ) {
    for( int y = 0; y < SCREEN_HEIGHT; y++ ) {
      float x_dir = x - ( SCREEN_WIDTH / (float) 2 );
      float y_dir = y - ( SCREEN_HEIGHT / (float) 2);

      vec4 direction = vec4( x_dir, y_dir, focal, 1.0);
      vec4 start = matrix * camera;
      // vec4 start = camera;
      Intersection c_i;

      if( ClosestIntersection( start, direction, c_i ) ){
        Triangle close = triangles[c_i.index];
        vec3 power = DirectLight( c_i );
        PutPixelSDL( screen, x, y, ( power + indirect_light ) * close.colour );
      }
    }
  }
  DrawBoundedBeams( screen );
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

void DrawBoundedBeams( screen* screen ){
  for( int i=0; i<beams.size(); i++ ){
    PhotonBeam b = beams[i];
    DrawBeam( screen, b );

    vec4 start = b.start;
    vec4 end = b.end;

    vec4 dir = b.start - b.end;
    float j=start.y;

    vec4 min_prev = vec4( start.x,
                          start.y,
                          start.z,
                          1.0f );

    while( j<end.y ){
      AABB beam_box;

      if( j + b.radius > end.y ){
        j = end.y;
      } else {
        j = j + b.radius;
      }

      beam_box.min = min_prev;
      float max_x = min_prev.x - ( ( min_prev.y - j ) * dir.x / dir.y );
      float max_z = min_prev.z - ( ( min_prev.y - j ) * dir.z / dir.y );
      beam_box.max = vec4( max_x, j, max_z, 1.0f );
      min_prev = beam_box.max;
      DrawBoundingBox( screen, beam_box );
    }
  }
  DrawBoundingBox( screen, root );
}

void DrawBeam( screen* screen, PhotonBeam& b ){
  mat4 matrix;  TransformationMatrix( matrix );

  Pixel proj1; Vertex v1; v1.position = b.start;
  Pixel proj2; Vertex v2; v2.position = b.end;
  VertexShader( v1, proj1 );
  VertexShader( v2, proj2 );
  DrawLine( screen, v1, v2 );
}

AABB CastPhotonBeams( int number ){
  vec4 min_point = vec4( m, m, -m, 1 );
  vec4 max_point = vec4( -m, -m, m, 1 );

  mat4 matrix;  TransformationMatrix( matrix );
  vec4 origin = matrix * light_position;
  vec4 centre = vec4( origin.x, origin.y + 0.5, origin.z, 1.0f );
  float radius = 0.4f;

  for( int i=0; i<number; i++ ){
    Intersection c_i;
    vec4 direction = FindDirection( origin, centre, radius );
    direction = glm::normalize( direction );
    if( ClosestIntersection( origin, direction, c_i ) ){
      PhotonBeam beam;
      beam.start  = origin;
      beam.end    = c_i.position;
      beam.offset = 0;
      beam.radius = 0.01;
      beams.push_back( beam );

      max_point.x = fmax( beam.end.x, fmax( beam.start.x, max_point.x ) );
      max_point.y = fmax( beam.end.y, fmax( beam.start.y, max_point.y ) );
      max_point.z = fmin( beam.end.z, fmin( beam.start.z, max_point.z ) );
      min_point.x = fmin( beam.end.x, fmin( beam.start.x, max_point.x ) );
      min_point.y = fmin( beam.end.y, fmin( beam.start.y, max_point.y ) );
      min_point.z = fmax( beam.end.z, fmax( beam.start.z, max_point.z ) );

    } else {
      cout << "Direction does not terminate\n";
    }
  }

  AABB root;
  root.min = min_point;
  root.max = max_point;

  return root;
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
