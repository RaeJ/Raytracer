#ifndef TEST_MODEL_CORNEL_BOX_H
#define TEST_MODEL_CORNEL_BOX_H

// Defines a simple test model: The Cornel Box

#include <glm/glm.hpp>
#include <vector>

float MULT = ( 5.0f / 12.0f );

// Used to describe a triangular surface:
class Triangle
{
public:
	glm::vec4 v0;
	glm::vec4 v1;
	glm::vec4 v2;
	glm::vec4 normal;
	glm::vec3 colour;

	Triangle( glm::vec4 v0, glm::vec4 v1, glm::vec4 v2, glm::vec3 colour )
		: v0(v0), v1(v1), v2(v2), colour(colour)
	{
		ComputeNormal();
	}

	void ComputeNormal()
	{
	  glm::vec3 e1 = glm::vec3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
	  glm::vec3 e2 = glm::vec3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
	  glm::vec3 normal3 = glm::normalize( glm::cross( e2, e1 ) );
	  normal.x = normal3.x;
	  normal.y = normal3.y;
	  normal.z = normal3.z;
	  normal.w = 1.0;
	}
};

// Loads the Cornell Box. It is scaled to fill the volume:
// -1 <= x <= +1
// -1 <= y <= +1
// -1 <= z <= +1
void LoadTestModel( std::vector<Triangle>& triangles )
{
	using glm::vec3;
	using glm::vec4;

	// Defines colours:
	vec3 red(    0.75f, 0.15f, 0.15f );
	vec3 yellow( 0.75f, 0.75f, 0.15f );
	vec3 green(  0.15f, 0.75f, 0.15f );
	vec3 cyan(   0.15f, 0.75f, 0.75f );
	vec3 blue(   0.15f, 0.15f, 0.75f );
	vec3 purple( 0.75f, 0.15f, 0.75f );
	vec3 white(  0.75f, 0.75f, 0.75f );

	triangles.clear();
	triangles.reserve( 5*2*3 );

	// ---------------------------------------------------------------------------
	// Room

	float L = 555;			// Length of Cornell Box side.

	vec4 A(L,0,0,1);
	vec4 B(0,0,0,1);
	vec4 C(L,0,L,1);
	vec4 D(0,0,L,1);

	vec4 E(L,L,0,1);
	vec4 F(0,L,0,1);
	vec4 G(L,L,L,1);
	vec4 H(0,L,L,1);

	MULT = ( 5.0f / 12.0f );
	float W = L * MULT;

	vec4 I(W,L,L,1);
	vec4 J(L-W,L,L,1);
	vec4 K(W,L,0,1);
	vec4 M(L-W,L,0,1);

	vec4 N(W,L,L-W,1);
	vec4 O(L-W,L,L-W,1);
	vec4 P(W,L,W,1);
	vec4 Q(L-W,L,W,1);

	// // Ceiling
	// triangles.push_back( Triangle( E, M, G, white ) );
	// triangles.push_back( Triangle( F, H, I, white ) );
	// triangles.push_back( Triangle( K, F, I, white ) );
	// triangles.push_back( Triangle( M, J, G, white ) );
	// triangles.push_back( Triangle( N, I, J, white ) );
	// triangles.push_back( Triangle( J, O, N, white ) );
	// triangles.push_back( Triangle( M, K, P, white ) );
	// triangles.push_back( Triangle( P, Q, M, white ) );

	// Ceiling
	triangles.push_back( Triangle( E, F, G, white ) );
	triangles.push_back( Triangle( F, H, G, white ) );

	// Floor:
	triangles.push_back( Triangle( C, B, A, white ) );
	triangles.push_back( Triangle( C, D, B, white ) );

	// Left wall
	triangles.push_back( Triangle( A, E, C, green ) );
	triangles.push_back( Triangle( C, E, G, green ) );

	// Right wall
	triangles.push_back( Triangle( F, B, D, red ) );
	triangles.push_back( Triangle( H, F, D, red ) );

	// Back wall
	triangles.push_back( Triangle( G, D, C, white ) );
	triangles.push_back( Triangle( G, H, D, white ) );

	// ---------------------------------------------------------------------------
	// Short block

	A = vec4(290,0,114,1);
	B = vec4(130,0, 65,1);
	C = vec4(240,0,272,1);
	D = vec4( 82,0,225,1);

	E = vec4(290,165,114,1);
	F = vec4(130,165, 65,1);
	G = vec4(240,165,272,1);
	H = vec4( 82,165,225,1);

	// Front
	triangles.push_back( Triangle(E,B,A,red) );
	triangles.push_back( Triangle(E,F,B,red) );

	// Front
	triangles.push_back( Triangle(F,D,B,red) );
	triangles.push_back( Triangle(F,H,D,red) );

	// BACK
	triangles.push_back( Triangle(H,C,D,red) );
	triangles.push_back( Triangle(H,G,C,red) );

	// LEFT
	triangles.push_back( Triangle(G,E,C,red) );
	triangles.push_back( Triangle(E,A,C,red) );

	// TOP
	triangles.push_back( Triangle(G,F,E,red) );
	triangles.push_back( Triangle(G,H,F,red) );

	// ---------------------------------------------------------------------------
	// Tall block

	A = vec4(423,0,247,1);
	B = vec4(265,0,296,1);
	C = vec4(472,0,406,1);
	D = vec4(314,0,456,1);

	E = vec4(423,330,247,1);
	F = vec4(265,330,296,1);
	G = vec4(472,330,406,1);
	H = vec4(314,330,456,1);

	// Front
	triangles.push_back( Triangle(E,B,A,blue) );
	triangles.push_back( Triangle(E,F,B,blue) );

	// Front
	triangles.push_back( Triangle(F,D,B,blue) );
	triangles.push_back( Triangle(F,H,D,blue) );

	// BACK
	triangles.push_back( Triangle(H,C,D,blue) );
	triangles.push_back( Triangle(H,G,C,blue) );

	// LEFT
	triangles.push_back( Triangle(G,E,C,blue) );
	triangles.push_back( Triangle(E,A,C,blue) );

	// TOP
	triangles.push_back( Triangle(G,F,E,blue) );
	triangles.push_back( Triangle(G,H,F,blue) );


	// ----------------------------------------------
	// Scale to the volume [-1,1]^3

	for( size_t i=0; i<triangles.size(); ++i )
	{
		triangles[i].v0 *= 2/L;
		triangles[i].v1 *= 2/L;
		triangles[i].v2 *= 2/L;

		triangles[i].v0 -= vec4(1,1,1,1);
		triangles[i].v1 -= vec4(1,1,1,1);
		triangles[i].v2 -= vec4(1,1,1,1);

		triangles[i].v0.x *= -1;
		triangles[i].v1.x *= -1;
		triangles[i].v2.x *= -1;

		triangles[i].v0.y *= -1;
		triangles[i].v1.y *= -1;
		triangles[i].v2.y *= -1;

		triangles[i].v0.w = 1.0;
		triangles[i].v1.w = 1.0;
		triangles[i].v2.w = 1.0;

		triangles[i].ComputeNormal();
	}
}

#endif
