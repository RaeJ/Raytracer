#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;


#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false


/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */

vector<Triangle> triangles;

float m = std::numeric_limits<float>::max();
const float focal = SCREEN_WIDTH * 2;

vec4 camera(0, 0, -3, 1.0);
vec3 theta( 0.0, 0.0, 0.0 );
vec4 light_position(0,-0.5,-0.7,1);
vec3 light_power = 14.f * vec3( 1, 1, 1 );
vec3 indirect_light = 0.5f*vec3( 1, 1, 1 );
#define PI 3.14159265

/* ----------------------------------------------------------------------------*/
/* STRUCTS                                                                     */

struct Intersection
{
  vec4 position;
  float distance;
  int index;
};

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


int main( int argc, char* argv[] )
{
  LoadTestModel( triangles );

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

/*Place your drawing here*/
void Draw(screen* screen)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  for( int x = 0; x < SCREEN_WIDTH; x++ ) {
    for( int y = 0; y < SCREEN_HEIGHT; y++ ) {
      float x_dir = x - ( SCREEN_WIDTH / (float) 2 );
      float y_dir = y - ( SCREEN_HEIGHT / (float) 2);

      vec4 direction = vec4( x_dir, y_dir, focal, 1.0);
      vec4 start = camera;
      Intersection c_i;

      if( ClosestIntersection( start, direction, c_i ) ){
        Triangle close = triangles[c_i.index];
        vec3 power = DirectLight( c_i );
        PutPixelSDL( screen, x, y, power * close.colour );
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
  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
  UserInput();
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

  vec3 radius = vec3 ( light_location ) - vec3( i.position );
  float A = 4 * PI * glm::dot( radius, radius );
  vec3 normal = vec3( triangles[i.index].normal );
  float r_dot_n = glm::dot( glm::normalize( radius ), glm::normalize( normal ) );
  r_dot_n = max( r_dot_n, 0.0f );

  vec4 direction = glm::normalize( vec4( radius, 1.0f ) );
  vec4 start = i.position + 0.001f * direction;
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
      camera.z += 0.005;
    }
    if( keystate[SDL_SCANCODE_S] ) {
      camera.z -= 0.005;
    }
    if( keystate[SDL_SCANCODE_A] ) {
      camera.x -= 0.005;
    }
    if( keystate[SDL_SCANCODE_D] ) {
      camera.x += 0.005;
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
    // Reset state
    if( keystate[SDL_SCANCODE_R] ) {
      camera = vec4( 0, 0, -3.00, 1 );
      theta = vec3( 0.0, 0.0, 0.0 );
      light_position = vec4(0,-0.5,-0.7,1);
    }
  }
