#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <iostream>
#include <fstream>
#include "Intersections.h"
#include "Control.h"

struct BasicIntersection
{
  vec3 entry_point;
  vec3 exit_point;
};

float analysis_extinction = EXT_ANALYSIS;
float mfp = 1/analysis_extinction;

float Tr( float t );
float TrPrime( float t_min, float t_plus );
void RunAnalysis();
void SetUpAnalysis( vec3& a, vec3& c, vec3& omega_a, vec3& omega_c,
                    float& t_a, float& t_c );
bool CylinderIntersection( const vec3 a, const vec3 a_omega,
                           const vec3 c, const vec3 c_omega,
                           const float kernel_width,
                           BasicIntersection& intersection );
bool SphereIntersection( const vec3 p, const vec3 dir,
                         const float kernel_width,
                         const vec3 centre,
                         BasicIntersection& intersection );
void BsBs2D1( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file );
void BsBl2D2( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file );
void BsBs1D( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file );
void BsBl1D( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file );
void BlBl2D1( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file );
void PBl3D( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file );
void BlP3D( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file );
void BlBs1D( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file );


void RunAnalysis(){
  std::ofstream myfile;
  myfile.open ("results.csv");

  vec3 a, c, omega_a, omega_c;
  float t_a, t_c;
  SetUpAnalysis( a, c, omega_a, omega_c, t_a, t_c );

  float delta_width = 0.02 * mfp;

  for( float kernel_width = delta_width; kernel_width < ( 5 * mfp ); kernel_width = kernel_width + delta_width ){
    myfile << ( kernel_width / mfp );
    BsBs2D1( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, myfile );
    BsBs1D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, myfile );
    BsBl2D2( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, myfile );
    PBl3D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, myfile );
    BsBl1D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, myfile );
    BlBs1D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, myfile );
    // BlP3D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, myfile );
    myfile << "\n";
  }
  myfile.close();
}

void BlP3D( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file ){
  BasicIntersection intersect1;
  vec3 point         = c + ( t_c * omega_c );
  BasicIntersection intersect2;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect1 ) &&
      SphereIntersection( a, omega_a, kernel_width, point, intersect2 ) ){
    float ta_minus    = glm::length( a - intersect2.entry_point );
    float ta_plus     = glm::length( a - intersect2.exit_point );

    float tc_minus    = t_c - kernel_width;
    float tc_plus     = t_c + kernel_width;


    // NOTE: does not include the phase
    float expectation = pow( kernel_width, -dimension ) *
                        TrPrime( ta_minus, ta_plus ) * TrPrime( tc_minus, tc_plus );
    float variance    = pow( kernel_width, -dimension * 2 ) *
                        ( ( pow( TrPrime( ta_minus, ta_plus ), 2 ) *
                        ( TrPrime( tc_minus, tc_plus ) / analysis_extinction ) ) -
                        ( pow( TrPrime( ta_minus, ta_plus ), 2 ) * pow( TrPrime( tc_minus, tc_plus ), 2 ) ) );

    float nsd         = sqrtf( variance ) / expectation;

    file << "," << to_string( nsd );
  } else {
    file << "," ;
  }
}

void BlBs1D( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){

    // NOTE: does not include the phase
    float expectation = pow( kernel_width, -dimension ) *
                        Tr( t_a ) * Tr( t_c );
    float variance    = pow( kernel_width, -dimension * 2 ) *
                        ( ( pow( Tr( t_a ), 2 ) * Tr( t_c ) ) -
                        ( pow( Tr( t_a ), 2 ) * pow( Tr( t_c ), 2 ) ) );

    float nsd         = sqrtf( variance ) / expectation;

    file << "," << to_string( nsd );
  } else {
    file << "," ;
  }
}

void BsBl1D( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){

    // NOTE: does not include the phase
    float expectation = pow( kernel_width, -dimension ) *
                        Tr( t_a ) * Tr( t_c );
    float variance    = pow( kernel_width, -dimension * 2 ) *
                        ( ( Tr( t_a ) * pow( Tr( t_c ), 2 ) ) -
                        ( pow( Tr( t_a ), 2 ) * pow( Tr( t_c ), 2 ) ) );

    float nsd         = sqrtf( variance ) / expectation;

    file << "," << to_string( nsd );
  } else {
    file << "," ;
  }
}

void PBl3D( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file ){
  BasicIntersection intersect;
  vec3 centre = a + ( t_a * omega_a );
  if( SphereIntersection( c, omega_c, kernel_width, centre, intersect ) ){
    float tc_minus = glm::length( intersect.entry_point - c );
    float tc_plus  = glm::length( intersect.exit_point - c );

    vec3 midpoint  = ( intersect.entry_point + intersect.exit_point ) / 2.0f;
    float distance = glm::length( centre - midpoint );

    float ta_minus = t_a - distance;
    float ta_plus  = t_a + distance;

    float expectation = pow( kernel_width, -dimension ) *
                        TrPrime( ta_minus, ta_plus ) *
                        TrPrime( tc_minus, tc_plus );

    float variance    = pow( kernel_width, -dimension * 2 ) *
                        ( ( ( TrPrime( ta_minus, ta_plus ) / analysis_extinction ) *
                        TrPrime( tc_minus, tc_plus ) / analysis_extinction ) -
                        ( TrPrime( ta_minus, ta_plus ) * TrPrime( tc_minus, tc_plus ) ) );

    float nsd         = sqrtf( variance ) / expectation;

    file << "," << to_string( nsd );
  } else {
    file << "," ;
  }
}

void BsBl2D2( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){
    // NOTE: I am not sure that this is the way to do it
    float ta_minus = glm::length( intersect.entry_point - c );
    float ta_plus  = glm::length( intersect.exit_point - c );

    // NOTE: does not include the phase
    float expectation = pow( kernel_width, -dimension ) *
                        TrPrime( ta_minus, ta_plus ) *
                        Tr( t_c );
    float variance    = pow( kernel_width, -dimension * 2 ) *
                        ( pow( Tr( t_c ), 2 ) * ( 2 / pow( analysis_extinction, 2 ) ) *
                        ( Tr( ta_minus ) + ( Tr( ta_plus ) *
                        ( ( ( ta_minus - ta_plus ) * analysis_extinction ) - 1 ) ) ) -
                        ( pow( Tr( t_c ), 2 ) * pow( TrPrime( ta_minus, ta_plus ), 2 ) ) );

    float nsd         = sqrtf( variance ) / expectation;

    file << "," << to_string( nsd );
  } else {
    file << "," ;
  }
}

void BsBs1D( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){

    // NOTE: does not include the phase
    float expectation = pow( kernel_width, -dimension ) *
                        Tr( t_a ) * Tr( t_c );
    float variance    = pow( kernel_width, -dimension * 2 ) *
                        ( ( Tr( t_a ) * Tr( t_c ) ) -
                        ( pow( Tr( t_a ), 2 ) * pow( Tr( t_c ), 2 ) ) );

    float nsd         = sqrtf( variance ) / expectation;

    file << "," << to_string( nsd );
  } else {
    file << "," ;
  }
}

void BlBl2D1( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){

    float nsd         = 0;

    file << "," << to_string( nsd );
  } else {
    file << "," ;
  }
}

void BsBs2D1( const float kernel_width, const int dimension,
            const vec3 a, const vec3 omega_a, const float t_a,
            const vec3 c, const vec3 omega_c, const float t_c,
            std::ofstream& file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){
    float tc_minus = glm::length( intersect.entry_point - c );
    float tc_plus  = glm::length( intersect.exit_point - c );

    // NOTE: does not include the phase
    float expectation = pow( kernel_width, -dimension ) *
                        Tr( t_a ) *
                        TrPrime( tc_minus, tc_plus );
    float variance    = pow( kernel_width, -dimension * 2 ) *
                        ( Tr( t_a ) * ( 2 / pow( analysis_extinction, 2 ) ) *
                        ( Tr( tc_minus ) + ( Tr( tc_plus ) *
                        ( ( ( tc_minus - tc_plus ) * analysis_extinction ) -1 ) ) ) -
                        ( pow( Tr( t_a ), 2 ) * pow( TrPrime( tc_minus, tc_plus ), 2 ) ) );

    float nsd         = sqrtf( variance ) / expectation;

    file << "," << to_string( nsd );
  } else {
    file << "," ;
  }
}

void SetUpAnalysis( vec3& a, vec3& c, vec3& omega_a, vec3& omega_c,
                    float& t_a, float& t_c ){

  vec3 view_point    = vec3( 0, 0, 0 );
  vec3 closest_point = vec3( view_point.x + ( 0.25 * mfp ),
                             view_point.y,
                             view_point.z + ( 10 * mfp )
                           );
  vec3 beam_origin   = vec3( closest_point.x + ( 0.25 * mfp ),
                             closest_point.y - ( 5 * mfp ),
                             closest_point.z
                           );
  vec3 view_dir      = vec3( 0, 0, 1 );
  vec3 beam_dir      = vec3( 0, 1, 0 );

  c = view_point;   omega_c = view_dir;
  a = beam_origin;  omega_a = beam_dir;

  t_c = glm::length( view_point - closest_point);
  t_a = glm::length( beam_origin - closest_point);
}

float Tr( float length ){
  float power     = - analysis_extinction * length;
  return exp( power );
}

float TrPrime( float t_min, float t_plus ){
  float numerator = Tr( t_min ) - Tr( t_plus );
  return ( numerator / analysis_extinction );
}

bool SphereIntersection( const vec3 p, const vec3 dir,
                           const float kernel_width,
                           const vec3 centre,
                           BasicIntersection& intersection ){
  vec3 vpc = centre - p;  // this is the vector from p to c
  float cos_theta = glm::dot( vpc, dir ) /
                    ( glm::length( vpc ) * glm::length( dir ) );
  float distance  = cos_theta * glm::length( vpc );
  vec3 pc         = p + ( distance * dir );

  if ( glm::dot( vpc, dir ) < 0 ){
    if ( glm::length( vpc ) > kernel_width ){
      return false;
    }
    else if ( glm::length( vpc ) == kernel_width ){
      intersection.entry_point = p;
      intersection.exit_point  = p;
    } else {
      float dist = sqrt( pow( kernel_width, 2 ) - pow( glm::length( pc - centre ), 2 ) );
      float di1  = dist - glm::length( pc - p );
      intersection.entry_point = p;
      intersection.exit_point  = p + ( dir * di1 );
    }
  } else{
    if( glm::length( centre - pc ) > kernel_width ){
      return false;
    } else {
      float dist = sqrt( pow( kernel_width, 2 ) - pow( glm::length( pc - centre ), 2 ) );
      float di1;
      if( glm::length( vpc ) > kernel_width ){
        di1 = glm::length( pc - p ) - dist;
      } else {
        di1 = glm::length( pc - p ) + dist;
      }
      intersection.entry_point = p + ( dir * di1 );
      float di2                = di1 + ( 2 *
                                 glm::length( pc - intersection.entry_point ) );
      intersection.exit_point  = p + ( dir * di2 );
    }
  }
  return true;
}

bool CylinderIntersection( const vec3 a, const vec3 a_omega,
                           const vec3 c, const vec3 c_omega,
                           const float kernel_width,
                           BasicIntersection& intersection ){

    bool first_inte = false;

    vec3 beam_start = a;
    vec3 beam_end   = a + ( a_omega * 10.0f * mfp );

    vec3 difference = beam_end - beam_start;
    vec3 w_unit_dir = glm::normalize( vec3( 0.0f,
                                            glm::length( difference ),
                                            0.0f ) );

    float S_inv     = 1/glm::length( difference ) ;

    vec3 c_unit_dir = difference * S_inv;

    mat3 R_inv      = findRotationMatrix( c_unit_dir, w_unit_dir );

    vec3 T_inv      = -( R_inv * a * S_inv );

    vec3 origin_prime    = ( R_inv * ( c * S_inv  ) ) + T_inv;
    vec3 dir_prime       = R_inv * c_omega;

    vec3 first_hit;
    vec3 second_hit;

    // Quadratic Formula
    float t0 = -1, t1 = -1;

    // a=xD2+yD2, b=2xExD+2yEyD, and c=xE2+yE2-1.
    float aq = dir_prime.x * dir_prime.x
             + dir_prime.z * dir_prime.z;

    float bq = 2 * origin_prime.x * dir_prime.x
             + 2 * origin_prime.z * dir_prime.z;

    float cq = origin_prime.x * origin_prime.x
             + origin_prime.z * origin_prime.z
             - pow( kernel_width * S_inv , 2 );

    float b24ac = bq*bq - 4*aq*cq;

    if( b24ac < 0 ) return false;

    float sqb24ac = sqrtf(b24ac);
    t0 = (-bq + sqb24ac) / (2 * aq);
    t1 = (-bq - sqb24ac) / (2 * aq);

    if (t0>t1) {float tmp = t0;t0=t1;t1=tmp;}

    float y0 = origin_prime.y + t0 * dir_prime.y;
    float y1 = origin_prime.y + t1 * dir_prime.y;

    if ( y0 < 0 )
    {
      if( y1 < 0 ) return false;
      else
      {
        // hit the cap
        float th1 = t0 + (t1-t0) * y0 / (y0-y1);
        if ( th1 <= 0 ) return false;
        first_hit = origin_prime + ( dir_prime * th1 );
        first_inte = true;

        if( y1 <= 1 ){
          second_hit = origin_prime + ( dir_prime * t1 );
        } else {
          float th2 = t1 + (t1-t0) * (y1-1) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
          }
        }
      }
    }
    else if ( y0 >= 0 && y0 <= 1 )
    {
      // hit the cylinder bit
      if( t0 <= 0 ) return false;
      first_hit = origin_prime + ( dir_prime * t0 );
      first_inte = true;

      if( y1 >= 0 ){
        if( y1 <= 1 ){
          second_hit = origin_prime + ( dir_prime * t1 );
        } else {
          float th2 = t1 + (t1-t0) * (y1-1) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
          }
        }
      } else {
        float th2 = t0 + (t1-t0) * (y0) / (y0-y1);
        if (th2>0){
          second_hit = origin_prime + ( dir_prime * th2 );
        }
      }
    }
    else if ( y0 > 1 )
    {
      if ( y1 > 1 ) return false;
      else {
        // hit the cap
        float th = t0 + (t1-t0) * (y0-1) / (y0-y1);
        if( th <= 0 ) return false;
        first_hit = origin_prime + ( dir_prime * th );
        first_inte = true;
        if( y1 >= 0 ){
          second_hit = origin_prime + ( dir_prime * t1 );
        } else {
          float th2 = t0 + (t1-t0) * (y0) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
          }
        }
      }
    }
    intersection.entry_point = ( glm::inverse( R_inv ) * ( first_hit - T_inv ) ) / S_inv;
    intersection.exit_point  = ( glm::inverse( R_inv ) * ( second_hit - T_inv ) ) / S_inv;

    return first_inte;
}

#endif
