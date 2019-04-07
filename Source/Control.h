#ifndef CONTROL_H
#define CONTROL_H

#define SCREEN_WIDTH 512
#define SCREEN_HEIGHT 512
#define FULLSCREEN_MODE false
#define SSAA 1

#define PHOTON_NUMBER 500
#define BOUNCES 20
#define ADAPTIVE false

// TODO: Should fix absorbed at some point
#define ABSORBED 0.0f

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

vec4 light_position(0,-0.98,-0.2,1);
vec3 light_power = 0.001f * vec3( 1, 1, 1 );
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
