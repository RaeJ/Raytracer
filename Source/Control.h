#ifndef CONTROL_H
#define CONTROL_H

#define SCREEN_WIDTH 512
#define SCREEN_HEIGHT 512
#define FULLSCREEN_MODE false
#define SSAA 1

#define PHOTON_NUMBER 500
#define BOUNCES 0
#define ADAPTIVE false

#define SHORT_BEAMS false
#define SHORT_VIEW false

#define SCATTER true

#define ONE_DIMENSIONAL true

#define RUN_ANALYSIS false
#define EXT_ANALYSIS 0.04f

#define FIXED_RADIUS false
#define RADIUS 0.01f

#define HETEROGENEOUS false

// TODO: Should fix absorbed at some point
#define ABSORBED 0.0f

#define SEED 32
// #define SEED std::chrono::system_clock::now().time_since_epoch().count()

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

vec4 light_position(0,-0.8,-0.5,1);
// vec3 light_power = 0.00002f * vec3( 1, 1, 1 );
vec3 light_power = 1.5f * vec3( 1, 1, 1 );
  // NOTE: 0.025 is good
// vec3 light_power = 20.1f * vec3( 1, 1, 1 );

float absorption_c = 0.035;
float scattering_c = 0.005;
float extinction_c = absorption_c + scattering_c;

vec4 camera(0, 0, -3, 1.0);

// TODO: fix issue of power depending on Transmittance
// float absorption_c = 0.0005;
// float scattering_c = 0.00001;
// float extinction_c = absorption_c + scattering_c;

#endif
