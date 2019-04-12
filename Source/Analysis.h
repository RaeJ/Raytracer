#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <iostream>
#include <fstream>
#include "Intersections.h"
#include "Control.h"

struct BasicIntersection
{
  glm::dvec3 entry_point;
  glm::dvec3 exit_point;
};

double analysis_extinction = EXT_ANALYSIS;
double mfp = 1/analysis_extinction;

double Tr( double t );
double TrPrime( double t_min, double t_plus );
void RunAnalysis();
void SetUpAnalysis( glm::dvec3& a, glm::dvec3& c, glm::dvec3& omega_a, glm::dvec3& omega_c,
                    double& t_a, double& t_c );
bool CylinderIntersection( const glm::dvec3 a, const glm::dvec3 a_omega,
                           const glm::dvec3 c, const glm::dvec3 c_omega,
                           const double kernel_width,
                           BasicIntersection& intersection );
bool SphereIntersection( const glm::dvec3 p, const glm::dvec3 dir,
                         const double kernel_width,
                         const glm::dvec3 centre,
                         BasicIntersection& intersection );
void BsBs2D1( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd );
void BsBl2D2( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd );
void BsBs1D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd );
void BsBl1D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd );
void BlBl2D1( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd );
void PBl3D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd );
void BlP3D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd );
void BlBs1D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd );
void PBs2D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd );
void rBsBs2D1( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& rrmse );
void rBlP3D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& rrmse );
void rBsBl1D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& rrmse );


void RunAnalysis(){
  std::ofstream nsd;
  nsd.open("NSD.csv");

  glm::dvec3 a, c, omega_a, omega_c;
  double t_a, t_c;
  SetUpAnalysis( a, c, omega_a, omega_c, t_a, t_c );

  double maximum_width = 5 * mfp;
  double delta_width   = 0.02 * mfp;

  for( double kernel_width = delta_width; kernel_width < maximum_width; kernel_width = kernel_width + delta_width ){
    nsd << ( kernel_width / mfp );
    BsBs2D1( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, nsd );
    BsBs1D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, nsd );
    BsBl2D2( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, nsd );
    PBl3D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, nsd );
    BsBl1D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, nsd );
    BlBs1D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, nsd );
    BlP3D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, nsd );
    PBs2D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, nsd );
    nsd << "\n";
  }
  nsd.close();

  std::ofstream rrmse;
  rrmse.open("rRMSE.csv");

  for( double kernel_width = 1.0e-6 * mfp ; kernel_width < maximum_width; kernel_width = kernel_width * 1.05 ){
    rrmse << ( kernel_width / mfp );
    rBsBs2D1( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, rrmse );
    // rBsBs1D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, rrmse );
    // rBsBl2D2( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, rrmse );
    // rPBl3D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, rrmse );
    rBsBl1D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, rrmse );
    // rBlBs1D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, rrmse );
    rBlP3D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, rrmse );
    // rPBs2D( kernel_width, 2, a, omega_a, t_a, c, omega_c, t_c, rrmse );
    rrmse << "\n";
  }
  rrmse.close();
}
// TODO: work out if the rFunctions are correctly working

void rBsBl1D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& rrmse_file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){

    // NOTE: does not include the phase
    double expectation = pow( kernel_width, -dimension ) *
                        Tr( t_a ) * Tr( t_c );
    double variance    = pow( kernel_width, -dimension * 2 ) *
                        ( ( Tr( t_a ) * pow( Tr( t_c ), 2 ) ) -
                        ( pow( Tr( t_a ), 2 ) * pow( Tr( t_c ), 2 ) ) );

    double reference = Tr( t_a ) * Tr( t_c );
    double sqr_bias  = pow( ( expectation - reference ), 2 );
    double rrmse     = sqrtf( variance + ( sqr_bias / reference ) );

    rrmse_file << "," << to_string( rrmse );
  } else {
    rrmse_file << "," ;
  }
}

void rBlP3D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& rrmse_file ){
  BasicIntersection intersect;
  glm::dvec3 point         = c + ( t_c * omega_c );
  if( SphereIntersection( a, omega_a, kernel_width, point, intersect ) ){
    double ta_minus    = glm::length( a - intersect.entry_point );
    double ta_plus     = glm::length( a - intersect.exit_point );

    glm::dvec3 midpoint  = ( intersect.entry_point + intersect.exit_point ) / 2.0;
    double distance = glm::length( c - midpoint );

    double tc_minus = fmax( t_c - kernel_width, distance - kernel_width );
    double tc_plus  = fmin( t_c + kernel_width, distance + kernel_width );


    // NOTE: does not include the phase
    double expectation = pow( kernel_width, -dimension ) *
                        TrPrime( ta_minus, ta_plus ) * TrPrime( tc_minus, tc_plus );
    double variance    = pow( kernel_width, -dimension * 2 ) *
                        ( ( pow( TrPrime( ta_minus, ta_plus ), 2 ) *
                        ( TrPrime( tc_minus, tc_plus ) / analysis_extinction ) ) -
                        ( pow( TrPrime( ta_minus, ta_plus ), 2 ) * pow( TrPrime( tc_minus, tc_plus ), 2 ) ) );

    double reference = TrPrime( ta_minus, ta_plus ) * TrPrime( tc_minus, tc_plus );
    double sqr_bias  = pow( ( expectation - reference ), 2 );
    double rrmse     = sqrtf( variance + ( sqr_bias / reference ) );

    rrmse_file << "," << to_string( rrmse );
  } else {
    rrmse_file << "," ;
  }
}

void BlP3D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd_file ){
  BasicIntersection intersect;
  glm::dvec3 point         = c + ( t_c * omega_c );
  if( SphereIntersection( a, omega_a, kernel_width, point, intersect ) ){
    double ta_minus    = glm::length( a - intersect.entry_point );
    double ta_plus     = glm::length( a - intersect.exit_point );

    glm::dvec3 midpoint  = ( intersect.entry_point + intersect.exit_point ) / 2.0;
    double distance = glm::length( c - midpoint );

    double tc_minus = fmax( t_c - kernel_width, distance - kernel_width );
    double tc_plus  = fmin( t_c + kernel_width, distance + kernel_width );


    // NOTE: does not include the phase
    double expectation = pow( kernel_width, -dimension ) *
                        TrPrime( ta_minus, ta_plus ) * TrPrime( tc_minus, tc_plus );
    double variance    = pow( kernel_width, -dimension * 2 ) *
                        ( ( pow( TrPrime( ta_minus, ta_plus ), 2 ) *
                        ( TrPrime( tc_minus, tc_plus ) / analysis_extinction ) ) -
                        ( pow( TrPrime( ta_minus, ta_plus ), 2 ) * pow( TrPrime( tc_minus, tc_plus ), 2 ) ) );

    double nsd         = sqrtf( variance ) / expectation;

    nsd_file << "," << to_string( nsd );
  } else {

    nsd_file << "," ;
  }
}

void BlBs1D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd_file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){

    // NOTE: does not include the phase
    double expectation = pow( kernel_width, -dimension ) *
                        Tr( t_a ) * Tr( t_c );
    double variance    = pow( kernel_width, -dimension * 2 ) *
                        ( ( pow( Tr( t_a ), 2 ) * Tr( t_c ) ) -
                        ( pow( Tr( t_a ), 2 ) * pow( Tr( t_c ), 2 ) ) );

    double nsd         = sqrtf( variance ) / expectation;

    nsd_file << "," << to_string( nsd );
  } else {
    nsd_file << "," ;
  }
}

void BsBl1D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd_file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){

    // NOTE: does not include the phase
    double expectation = pow( kernel_width, -dimension ) *
                        Tr( t_a ) * Tr( t_c );
    double variance    = pow( kernel_width, -dimension * 2 ) *
                        ( ( Tr( t_a ) * pow( Tr( t_c ), 2 ) ) -
                        ( pow( Tr( t_a ), 2 ) * pow( Tr( t_c ), 2 ) ) );

    double nsd         = sqrtf( variance ) / expectation;

    nsd_file << "," << to_string( nsd );
  } else {
    nsd_file << "," ;
  }
}

void PBs2D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd_file ){
  BasicIntersection intersect;
  glm::dvec3 centre = a + ( t_a * omega_a );
  if( SphereIntersection( c, omega_c, kernel_width, centre, intersect ) ){
    glm::dvec3 midpoint  = ( intersect.entry_point + intersect.exit_point ) / 2.0;
    double distance = glm::length( a - midpoint );

    double ta_minus = fmax( t_a - kernel_width, distance - kernel_width );
    double ta_plus  = fmin( t_a + kernel_width, distance + kernel_width );

    double expectation = pow( kernel_width, -dimension ) *
                        TrPrime( ta_minus, ta_plus ) *
                        Tr( t_c );

    double variance    = pow( kernel_width, -dimension * 2 ) *
                        ( ( ( TrPrime( ta_minus, ta_plus ) / analysis_extinction ) *
                        Tr( t_c ) ) -
                        ( pow( TrPrime( ta_minus, ta_plus ), 2 ) *
                        pow( Tr( t_c ), 2 ) ) );

    double nsd         = sqrtf( variance ) / expectation;

    nsd_file << "," << to_string( nsd );
  } else {
    nsd_file << "," ;
  }
}

void PBl3D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd_file ){
  BasicIntersection intersect;
  glm::dvec3 centre = a + ( t_a * omega_a );
  if( SphereIntersection( c, omega_c, kernel_width, centre, intersect ) ){
    double tc_minus = glm::length( intersect.entry_point - c );
    double tc_plus  = glm::length( intersect.exit_point - c );

    glm::dvec3 midpoint  = ( intersect.entry_point + intersect.exit_point ) / 2.0;
    double distance = glm::length( a - midpoint );

    double ta_minus = fmax( t_a - kernel_width, distance - kernel_width );
    double ta_plus  = fmin( t_a + kernel_width, distance + kernel_width );

    double expectation = pow( kernel_width, -dimension ) *
                        TrPrime( ta_minus, ta_plus ) *
                        TrPrime( tc_minus, tc_plus );

    double variance    = pow( kernel_width, -dimension * 2 ) *
                        ( ( ( TrPrime( ta_minus, ta_plus ) / analysis_extinction ) *
                        pow( TrPrime( tc_minus, tc_plus ), 2 ) ) -
                        ( pow( TrPrime( ta_minus, ta_plus ), 2 ) *
                        pow( TrPrime( tc_minus, tc_plus ), 2 ) ) );

    double nsd         = sqrtf( variance ) / expectation;

    nsd_file << "," << to_string( nsd );
  } else {
    nsd_file << "," ;
  }
}

void BsBl2D2( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd_file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){
    // NOTE: I am not sure that this is the way to do it
    // double ta_minus = glm::length( intersect.entry_point - a );
    // double ta_plus  = glm::length( intersect.exit_point - a );

    double ta_minus = t_a - kernel_width;
    double ta_plus  = t_a + kernel_width;

    // NOTE: does not include the phase
    double expectation = pow( kernel_width, -dimension ) *
                        TrPrime( ta_minus, ta_plus ) *
                        Tr( t_c );
    double variance    = pow( kernel_width, -dimension * 2 ) *
                        ( pow( Tr( t_c ), 2 ) * ( 2 / pow( analysis_extinction, 2 ) ) *
                        ( Tr( ta_minus ) + ( Tr( ta_plus ) *
                        ( ( ( ta_minus - ta_plus ) * analysis_extinction ) - 1 ) ) ) -
                        ( pow( Tr( t_c ), 2 ) * pow( TrPrime( ta_minus, ta_plus ), 2 ) ) );

    double nsd         = sqrtf( variance ) / expectation;

    nsd_file << "," << to_string( nsd );
  } else {
    nsd_file << "," ;
  }
}

void BsBs1D( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd_file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){

    // NOTE: does not include the phase
    double expectation = pow( kernel_width, -dimension ) *
                        Tr( t_a ) * Tr( t_c );
    double variance    = pow( kernel_width, -dimension * 2 ) *
                        ( ( Tr( t_a ) * Tr( t_c ) ) -
                        ( pow( Tr( t_a ), 2 ) * pow( Tr( t_c ), 2 ) ) );

    double nsd         = sqrtf( variance ) / expectation;

    nsd_file << "," << to_string( nsd );
  } else {
    nsd_file << "," ;
  }
}

void BlBl2D1( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd_file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){

    double nsd         = 0;

    nsd_file << "," << to_string( nsd );
  } else {
    nsd_file << "," ;
  }
}

void rBsBs2D1( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& rrmse_file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){
    double tc_minus = glm::length( intersect.entry_point - c );
    double tc_plus  = glm::length( intersect.exit_point - c );
    if( tc_minus == tc_plus ){
      tc_minus = tc_minus - kernel_width;
      tc_plus  = tc_plus + kernel_width;
    }

    // NOTE: does not include the phase
    double expectation = pow( kernel_width, -dimension ) *
                        Tr( t_a ) *
                        TrPrime( tc_minus, tc_plus );
    double variance    = pow( kernel_width, -dimension * 2 ) *
                        ( Tr( t_a ) * ( 2 / pow( analysis_extinction, 2 ) ) *
                        ( Tr( tc_minus ) + ( Tr( tc_plus ) *
                        ( ( ( tc_minus - tc_plus ) * analysis_extinction ) -1 ) ) ) -
                        ( pow( Tr( t_a ), 2 ) * pow( TrPrime( tc_minus, tc_plus ), 2 ) ) );

    double reference = Tr( t_a ) * TrPrime( tc_minus, tc_plus );
    double sqr_bias  = pow( ( expectation - reference ), 2 );
    double rrmse     = sqrtf( variance + ( sqr_bias / reference ) );

    rrmse_file << "," << to_string( rrmse );
  } else {
    rrmse_file << "," ;
  }
}

void BsBs2D1( const double kernel_width, const int dimension,
            const glm::dvec3 a, const glm::dvec3 omega_a, const double t_a,
            const glm::dvec3 c, const glm::dvec3 omega_c, const double t_c,
            std::ofstream& nsd_file ){
  BasicIntersection intersect;
  if( CylinderIntersection( a, omega_a, c, omega_c, kernel_width, intersect ) ){
    double tc_minus = glm::length( intersect.entry_point - c );
    double tc_plus  = glm::length( intersect.exit_point - c );

    // NOTE: does not include the phase
    double expectation = pow( kernel_width, -dimension ) *
                        Tr( t_a ) *
                        TrPrime( tc_minus, tc_plus );
    double variance    = pow( kernel_width, -dimension * 2 ) *
                        ( Tr( t_a ) * ( 2 / pow( analysis_extinction, 2 ) ) *
                        ( Tr( tc_minus ) + ( Tr( tc_plus ) *
                        ( ( ( tc_minus - tc_plus ) * analysis_extinction ) -1 ) ) ) -
                        ( pow( Tr( t_a ), 2 ) * pow( TrPrime( tc_minus, tc_plus ), 2 ) ) );

    double nsd         = sqrtf( variance ) / expectation;

    nsd_file << "," << to_string( nsd );
  } else {
    nsd_file << "," ;
  }
}

void SetUpAnalysis( glm::dvec3& a, glm::dvec3& c, glm::dvec3& omega_a, glm::dvec3& omega_c,
                    double& t_a, double& t_c ){

  glm::dvec3 view_point    = glm::dvec3( 0, 0, 0 );
  glm::dvec3 closest_point = glm::dvec3( view_point.x + ( 0.0 * mfp ),
                             view_point.y,
                             view_point.z + ( 10 * mfp )
                           );
  glm::dvec3 beam_origin   = glm::dvec3( closest_point.x + ( 0.0 * mfp ),
                             closest_point.y - ( 5 * mfp ),
                             closest_point.z
                           );
  glm::dvec3 view_dir      = glm::dvec3( 0, 0, 1 );
  glm::dvec3 beam_dir      = glm::dvec3( 0, 1, 0 );

  c = view_point;   omega_c = view_dir;
  a = beam_origin;  omega_a = beam_dir;

  t_c = glm::length( view_point - closest_point);
  t_a = glm::length( beam_origin - closest_point);
}

double Tr( double length ){
  double power     = - analysis_extinction * length;
  return exp( power );
}

double TrPrime( double t_min, double t_plus ){
  double numerator = Tr( t_min ) - Tr( t_plus );
  return ( numerator / analysis_extinction );
}

bool SphereIntersection( const glm::dvec3 p, const glm::dvec3 dir,
                           const double kernel_width,
                           const glm::dvec3 centre,
                           BasicIntersection& intersection ){
  glm::dvec3 vpc = centre - p;  // this is the vector from p to c
  double cos_theta = glm::dot( vpc, dir ) /
                    ( glm::length( vpc ) * glm::length( dir ) );
  double distance  = cos_theta * glm::length( vpc );
  glm::dvec3 pc         = p + ( distance * dir );

  if ( glm::dot( vpc, dir ) < 0 ){
    if ( glm::length( vpc ) > kernel_width ){
      return false;
    }
    else if ( glm::length( vpc ) == kernel_width ){
      intersection.entry_point = p;
      intersection.exit_point  = p;
    } else {
      double dist = sqrt( pow( kernel_width, 2 ) - pow( glm::length( pc - centre ), 2 ) );
      double di1  = dist - glm::length( pc - p );
      intersection.entry_point = p;
      intersection.exit_point  = p + ( dir * di1 );
    }
  } else{
    if( glm::length( centre - pc ) > kernel_width ){
      return false;
    } else {
      double dist = sqrt( pow( kernel_width, 2 ) - pow( glm::length( pc - centre ), 2 ) );
      double di1;
      if( glm::length( vpc ) > kernel_width ){
        di1 = glm::length( pc - p ) - dist;
      } else {
        di1 = glm::length( pc - p ) + dist;
      }
      intersection.entry_point = p + ( dir * di1 );
      double di2                = di1 + ( 2 *
                                 glm::length( pc - intersection.entry_point ) );
      intersection.exit_point  = p + ( dir * di2 );
    }
  }
  return true;
}

bool CylinderIntersection( const glm::dvec3 a, const glm::dvec3 a_omega,
                           const glm::dvec3 c, const glm::dvec3 c_omega,
                           const double kernel_width,
                           BasicIntersection& intersection ){

    bool first_inte = false;

    glm::dvec3 beam_start = a;
    glm::dvec3 beam_end   = a + ( a_omega * 10.0 * mfp );

    glm::dvec3 difference = beam_end - beam_start;
    glm::dvec3 w_unit_dir = glm::normalize( glm::dvec3( 0.0,
                                            glm::length( difference ),
                                            0.0 ) );

    double S_inv     = 1/glm::length( difference ) ;

    glm::dvec3 c_unit_dir = difference * S_inv;

    glm::dmat3 R_inv      = findRotationMatrix( c_unit_dir, w_unit_dir );

    glm::dvec3 T_inv      = -( R_inv * a * S_inv );

    glm::dvec3 origin_prime    = ( R_inv * ( c * S_inv  ) ) + T_inv;
    glm::dvec3 dir_prime       = R_inv * c_omega;

    glm::dvec3 first_hit;
    glm::dvec3 second_hit;

    // Quadratic Formula
    double t0 = -1, t1 = -1;

    // a=xD2+yD2, b=2xExD+2yEyD, and c=xE2+yE2-1.
    double aq = dir_prime.x * dir_prime.x
             + dir_prime.z * dir_prime.z;

    double bq = 2 * origin_prime.x * dir_prime.x
             + 2 * origin_prime.z * dir_prime.z;

    double cq = origin_prime.x * origin_prime.x
             + origin_prime.z * origin_prime.z
             - pow( kernel_width * S_inv , 2 );

    double b24ac = bq*bq - 4*aq*cq;

    if( b24ac < 0 ) return false;

    double sqb24ac = sqrtf(b24ac);
    t0 = (-bq + sqb24ac) / (2 * aq);
    t1 = (-bq - sqb24ac) / (2 * aq);

    if (t0>t1) {double tmp = t0;t0=t1;t1=tmp;}

    double y0 = origin_prime.y + t0 * dir_prime.y;
    double y1 = origin_prime.y + t1 * dir_prime.y;

    if ( y0 < 0 )
    {
      if( y1 < 0 ) return false;
      else
      {
        // hit the cap
        double th1 = t0 + (t1-t0) * y0 / (y0-y1);
        if ( th1 <= 0 ) return false;
        first_hit = origin_prime + ( dir_prime * th1 );
        first_inte = true;

        if( y1 <= 1 ){
          second_hit = origin_prime + ( dir_prime * t1 );
        } else {
          double th2 = t1 + (t1-t0) * (y1-1) / (y0-y1);
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
          double th2 = t1 + (t1-t0) * (y1-1) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
          }
        }
      } else {
        double th2 = t0 + (t1-t0) * (y0) / (y0-y1);
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
        double th = t0 + (t1-t0) * (y0-1) / (y0-y1);
        if( th <= 0 ) return false;
        first_hit = origin_prime + ( dir_prime * th );
        first_inte = true;
        if( y1 >= 0 ){
          second_hit = origin_prime + ( dir_prime * t1 );
        } else {
          double th2 = t0 + (t1-t0) * (y0) / (y0-y1);
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
