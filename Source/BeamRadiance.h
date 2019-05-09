#ifndef BEAM_RADIANCE_H
#define BEAM_RADIANCE_H

#include "CastPhotons.h"

void BeamRadiance( screen* screen,
                   vec4 start,
                   vec4 dir,
                   const Intersection& limit,
                   Node* parent,
                   glm::dvec3& current,
                   vector<PhotonBeam>& beams_const,
                   vector<PhotonBeam> beams_scatt,
                   int row
                 );
void BeamRadianceStandard( screen* screen,
                  vec4 start,
                  vec4 dir,
                  const Intersection& limit,
                  Node* parent,
                  glm::dvec3& current,
                  vector<PhotonBeam>& beams_const,
                  vector<PhotonBeam> beams_scatt,
                  int row
                );
void BeamRadianceHet( screen* screen,
                  vec4 start,
                  vec4 dir,
                  const Intersection& limit,
                  NodeGen* parent,
                  glm::dvec3& current,
                  vector<PhotonBeam>& beams_const,
                  vector<PhotonBeam> beams_scatt,
                  int row
                );
void BVHBeamRadianceHet( screen* screen,
                  vec4 start,
                  vec4 dir,
                  const Intersection& limit,
                  Node* parent,
                  glm::dvec3& current,
                  vector<PhotonBeam>& beams_const,
                  vector<PhotonBeam> beams_scatt,
                  int row
                );
double Integral_721( PhotonSeg s,
                    CylIntersection i,
                    float extinction,
                    vec4 dir  );
double Integral_721_ada( PhotonSeg s,
                         CylIntersection i,
                         float extinction,
                         vec4 dir );
double Integral_73( PhotonSeg s,
                    CylIntersection i,
                    float extinction,
                    vec4 dir  );
double Integral_73Het( PhotonSeg s,
                       CylIntersection i,
                       float extinction,
                       vec4 dir  );

double Integral_721_ada( PhotonSeg s, CylIntersection i, float extinction, vec4 dir ){
  vec3 beam_dir        = glm::normalize( vec3( s.end ) - vec3( s.start ) );
  vec3 camera_dir      = glm::normalize( vec3( dir ) );
  double cos_theta      = glm::dot( beam_dir, -camera_dir );
  double length_const   = glm::length( s.orig_start - s.end );

  double integrand = 0;
  double dt_c      = 0.001;

  for( double tc=i.tc_minus; tc<i.tc_plus; tc = tc + dt_c ){
    double tb  = i.tb_minus - ( abs( cos_theta ) * ( tc - i.tc_minus ) );
    double current_r   = ( s.radius / length_const ) * tb;
    // current_r = fmax( current_r, s.radius );
    double constant = Transmittance( tb, extinction );
    double transmitted = Transmittance( tc, extinction ) * constant
                        * ( scattering_c / pow( current_r, 2 ) ) * dt_c;
    // if( transmitted > 1e-6 ){
    integrand += transmitted;
    // }
  }
  return integrand;
}

 double Integral_721( PhotonSeg s, CylIntersection i, float extinction, vec4 dir ){
   vec3 beam_dir        = glm::normalize( vec3( s.end ) - vec3( s.start ) );
   vec3 camera_dir      = glm::normalize( vec3( dir ) );
   float cos_theta      = glm::dot( beam_dir, -camera_dir );

   // integral version

   // double integrand = 0;
   // double dt_c      = 0.0001;

   // for( double tc=i.tc_minus; tc<i.tc_plus; tc = tc + dt_c ){
   //   double tb  = i.tb_minus - ( abs( cos_theta ) * ( tc - i.tc_minus ) );
   //   double constant = Transmittance( tb, extinction );
   //   double transmitted = Transmittance( tc, extinction ) * constant * dt_c;
   //   integrand += transmitted;
   // }
   // return integrand;

   // analytical version
   double numerator = exp( -extinction * ( i.tc_minus - i.tc_plus ) *
                      ( abs( cos_theta ) - 1 ) ) - 1;
   double denominator = exp( extinction * ( i.tc_minus + i.tb_minus ) ) *
                        extinction * ( abs( cos_theta ) - 1 );
   return ( numerator / denominator );
 }

 double Integral_73( PhotonSeg s, CylIntersection i, float extinction, vec4 dir  ){
   double transmitted= 0;
   glm::dvec3 a    = vec3( s.start );   glm::dvec3 c = i.exit_point;
   glm::dvec3 b    = vec3( s.end );     glm::dvec3 d = i.entry_point;
   glm::dvec3 ab   = glm::normalize( b - a );
   glm::dvec3 cd   = glm::normalize( d - c );

   // glm::dvec3 ab_cross_cd = glm::cross( ab, cd );

   // PQ = a + t*ab - ( c + s*cd ) = a - c + ( t*ab - s*cd )
   // PQ = k( ab_cross_cd )
   // k( ab_cross_cd ) = a - c + ( t*ab - s*cd )

   double u_dot_u         = glm::dot( ab, ab );
   double u_dot_v         = glm::dot( ab, cd );
   double v_dot_v         = glm::dot( cd, cd );
   double u_dot_w         = glm::dot( ab, a - c );
   double v_dot_w         = glm::dot( cd, a - c );
   double denominator     = glm::dot( u_dot_u, v_dot_v ) -
                            glm::dot( u_dot_v, u_dot_v );

   if( ( denominator != 0 ) && ( u_dot_u != 0 ) && ( v_dot_v != 0 ) ){
     double t_ab = ( glm::dot( u_dot_v, v_dot_w ) - glm::dot( v_dot_v, u_dot_w ) ) / denominator;
     double t_cd = ( glm::dot( u_dot_u, v_dot_w ) - glm::dot( u_dot_v, u_dot_w ) ) / denominator;
     // double distance = glm::length( a - c + ( t_ab * ab ) - ( t_cd * cd ) );

     glm::dvec3 point_beam = a + ( t_ab * ab );
     glm::dvec3 point_view = c + ( t_cd * cd );

     double view_distance = glm::length( point_view );
     double beam_distance = glm::length( point_beam - glm::dvec3( s.orig_start ) );

     bool bound1 = false; bool bound2 = false;
     if( ( view_distance >= glm::length( d ) ) &&
         ( view_distance <= glm::length( c ) ) ){
       bound1 = true;
     }
     if( ( beam_distance >= glm::length( a - glm::dvec3( s.orig_start ) ) ) &&
         ( ( beam_distance <= glm::length( b - glm::dvec3( s.orig_start ) ) ) ) ){
       bound2 = true;
     }
     float scattering = scattering_c;


     if( bound1 && bound2 ){
       double t_bc       = glm::length( point_beam - glm::dvec3( s.orig_start ) );
       double t_cb       = view_distance;
       if( HETEROGENEOUS ){
         glm::length( b - glm::dvec3( s.orig_start ) );
         // float proportion = ( t_bc - glm::length( a - glm::dvec3( s.orig_start ) ) ) / t_bc;
         float proportion = ( t_bc - glm::length( a - glm::dvec3( s.orig_start ) ) ) /
                            ( glm::length( b - glm::dvec3( s.orig_start ) ) -
                              glm::length( a - glm::dvec3( s.orig_start ) ) );
         float additional = s.s_ext + ( ( s.e_ext - s.s_ext ) * proportion );

         float beam_extinction = s.c_ext + ( ( additional - s.c_ext ) / ( s.seg_id + 1 ) );

         vector<vec2> dist_ext;
         float view_extinction = Extinction3D( vec4( 0, 0, 0, 1 ),
                                               vec4( 0, 0, 0, 1 ) + ( dir * (float) t_cb ),
                                               GRID,
                                               dist_ext );
         // if( beam_extinction < 0 || view_extinction < 0  ||
         //     s.e_ext < 0 || s.e_ext < 0 || s.c_ext < 0 ){
         // // if( s.s_ext < 0 || s.e_ext < 0 || s.c_ext < 0 ){
         //   cout << "Beam extinction: " << beam_extinction << endl;
         //   cout << "View extinction: " << view_extinction << endl;
         //   cout << "Start ext: " << s.s_ext << endl;
         //   cout << "End ext: " << s.e_ext << endl;
         //   cout << "Current ext: " << s.c_ext << endl;
         //   cout << "Proportion: " << proportion << endl;
         // }
         transmitted       = Transmittance( t_bc, beam_extinction ) *
                             Transmittance( t_cb, view_extinction );

         // TODO: why did I have this commented out?
         scattering = ( beam_extinction / extinction_c ) * scattering_c;
       } else {
         transmitted       = Transmittance( t_bc, extinction ) *
                             Transmittance( t_cb, extinction );
       }
       if( transmitted < 1e-6 ){
         return 0;
       }

       double cos_theta  = glm::dot( ab, cd );
       double sin_theta  = sqrt( 1 - pow( cos_theta, 2 ) );
       double rad        = scattering / s.radius;
       double result = transmitted * rad / sin_theta;

       return ( result );
     } else {
       return 0;
     }
   } else {
     return 0;
   }
 }

 double Integral_73Het( PhotonSeg s, CylIntersection i, float extinction, vec4 dir  ){
   double transmitted= 0;
   glm::dvec3 a    = vec3( s.start );   glm::dvec3 c = i.exit_point;
   glm::dvec3 b    = vec3( s.end );     glm::dvec3 d = i.entry_point;
   glm::dvec3 ab   = glm::normalize( b - a );
   glm::dvec3 cd   = glm::normalize( d - c );

   double u_dot_u         = glm::dot( ab, ab );
   double u_dot_v         = glm::dot( ab, cd );
   double v_dot_v         = glm::dot( cd, cd );
   double u_dot_w         = glm::dot( ab, a - c );
   double v_dot_w         = glm::dot( cd, a - c );
   double denominator     = glm::dot( u_dot_u, v_dot_v ) -
                            glm::dot( u_dot_v, u_dot_v );

   if( ( denominator != 0 ) && ( u_dot_u != 0 ) && ( v_dot_v != 0 ) ){
     double t_ab = ( glm::dot( u_dot_v, v_dot_w ) - glm::dot( v_dot_v, u_dot_w ) ) / denominator;
     double t_cd = ( glm::dot( u_dot_u, v_dot_w ) - glm::dot( u_dot_v, u_dot_w ) ) / denominator;
     // double distance = glm::length( a - c + ( t_ab * ab ) - ( t_cd * cd ) );

     glm::dvec3 point_beam = a + ( t_ab * ab );
     glm::dvec3 point_view = c + ( t_cd * cd );

     double view_distance = glm::length( point_view );
     double beam_distance = glm::length( point_beam - glm::dvec3( s.orig_start ) );

     bool bound1 = false; bool bound2 = false;
     if( ( view_distance >= glm::length( d ) ) &&
         ( view_distance <= glm::length( c ) ) ){
       bound1 = true;
     }
     if( ( beam_distance >= glm::length( a - glm::dvec3( s.orig_start ) ) ) &&
         ( ( beam_distance <= glm::length( b - glm::dvec3( s.orig_start ) ) ) ) ){
       bound2 = true;
     }
     float scattering = scattering_c;


     if( bound1 && bound2 ){
       double t_bc       = glm::length( point_beam - glm::dvec3( s.orig_start ) );
       double t_cb       = view_distance;
       glm::length( b - glm::dvec3( s.orig_start ) );
       // float proportion = ( t_bc - glm::length( a - glm::dvec3( s.orig_start ) ) ) / t_bc;
       float proportion = ( t_bc - glm::length( a - glm::dvec3( s.orig_start ) ) ) /
                          ( glm::length( b - glm::dvec3( s.orig_start ) ) -
                            glm::length( a - glm::dvec3( s.orig_start ) ) );
       float additional = s.s_ext + ( ( s.e_ext - s.s_ext ) * proportion );

       float beam_extinction = s.c_ext + ( ( additional - s.c_ext ) / ( s.seg_id + 1 ) );

       vector<vec2> dist_ext;
       float view_extinction = Extinction3D( vec4( 0, 0, 0, 1 ),
                                             vec4( 0, 0, 0, 1 ) + ( dir * (float) t_cb ),
                                             GRID,
                                             dist_ext );

       transmitted       = Transmittance( t_bc, beam_extinction ) *
                           Transmittance( t_cb, view_extinction );

       // TODO: why did I have this commented out?
       scattering = ( beam_extinction / extinction_c ) * scattering_c;
       if( transmitted < 1e-6 ){
         return 0;
       }

       double cos_theta  = glm::dot( ab, cd );
       double sin_theta  = sqrt( 1 - pow( cos_theta, 2 ) );
       double rad        = scattering / s.radius;
       double result = transmitted * rad / sin_theta;

       return ( result );
     } else {
       return 0;
     }
   } else {
     return 0;
   }
 }

 void BeamRadianceHet( screen* screen,
                   vec4 start,
                   vec4 dir,
                   const Intersection& limit,
                   NodeGen* parent,
                   glm::dvec3& current,
                   vector<PhotonBeam>& beams_const,
                   vector<PhotonBeam> beams_scatt,
                   int row
                 ){
     vec4 hit;
     float max_distance  = glm::length( limit.position - start );
     AABB box = parent->aabb;
     if( HitBoundingBox( box, start, dir, hit ) ){
       // PositionShader( screen, hit, vec3( 1, 0, 1 ) );
       float hit_distance = glm::length( hit - start );
       if( hit_distance <= max_distance ){
         if( parent -> leaf ){
           PhotonSeg seg = parent->segment;
           if( seg.id != -1 ){
             CylIntersection intersect;
             if( HitCylinder( start, dir, seg, intersect ) ){
               if( intersect.valid ){
                 if( ONE_DIMENSIONAL ){
                   double _int     = Integral_73Het( seg,
                                                 intersect,
                                                 extinction_c,
                                                 dir );

                   double phase_f  = 1 / ( 4 * PI );
                   PhotonBeam beam;
                   if( seg.scattered ){
                     beam = beams_scatt[ seg.id ];
                   } else {
                     beam = beams_const[ seg.id ];
                   }

                   current        += beam.energy * phase_f * _int;
                 } else {
                   double _int     = Integral_721( seg,
                                                  intersect,
                                                  extinction_c,
                                                  dir );

                   double phase_f  = 1 / ( 4 * PI );
                   double rad      = scattering_c / ( PI * pow( seg.radius, 2 ) );

                   PhotonBeam beam;
                   if( seg.scattered ){
                     beam = beams_scatt[ seg.id ];
                   } else {
                     beam = beams_const[ seg.id ];
                   }
                   current        += beam.energy * phase_f * rad * _int;
                 }
               }
             }
           }
         }
         for( int i=0; i<(parent->child).size(); i++ ){
           BeamRadianceHet( screen, start, dir, limit, parent->child[i], current, beams_const, beams_scatt, row );
         }
       }
     }
 }

 void BVHBeamRadianceHet( screen* screen,
                   vec4 start,
                   vec4 dir,
                   const Intersection& limit,
                   Node* parent,
                   glm::dvec3& current,
                   vector<PhotonBeam>& beams_const,
                   vector<PhotonBeam> beams_scatt,
                   int row
                 ){
   vec4 hit;
   float max_distance  = glm::length( limit.position - start );
   AABB box = parent->aabb;
   if( HitBoundingBox( box, start, dir, hit ) ){
     // PositionShader( screen, hit, vec3( 1, 0, 1 ) );
     float hit_distance = glm::length( hit - start );
     if( hit_distance <= max_distance ){
       if( parent -> leaf ){
         PhotonSeg segments[2];
         segments[0] = parent->segments[0];
         segments[1] = parent->segments[1];
         for( unsigned int i = 0; i < sizeof(segments)/sizeof(segments[0]); i++ ){
           PhotonSeg seg = segments[i];
           if( seg.id != -1 ){
             CylIntersection intersect;
             if( HitCylinder( start, dir, seg, intersect ) ){
               if( intersect.valid ){
                 if( ONE_DIMENSIONAL ){
                   double _int     = Integral_73Het( seg,
                                                 intersect,
                                                 extinction_c,
                                                 dir );

                   double phase_f  = 1 / ( 4 * PI );
                   PhotonBeam beam;
                   if( seg.scattered ){
                     beam = beams_scatt[ seg.id ];
                   } else {
                     beam = beams_const[ seg.id ];
                   }

                   current        += beam.energy * phase_f * _int;
                 } else {
                   cout << "Heterogeneous should be 1D" << endl;
                 }
               }
             }
           }
         }
       }
       if( parent->left != NULL ){
         BVHBeamRadianceHet( screen, start, dir, limit, parent->left, current, beams_const, beams_scatt, row );
       }
       if( parent->right != NULL ){
         BVHBeamRadianceHet( screen, start, dir, limit, parent->right, current, beams_const, beams_scatt, row );
       }
     }
   }
}
void BeamRadianceStandard( screen* screen,
                  vec4 start,
                  vec4 dir,
                  const Intersection& limit,
                  Node* parent,
                  glm::dvec3& current,
                  vector<PhotonBeam>& beams_const,
                  vector<PhotonBeam> beams_scatt,
                  int row
                ){
  vec4 hit;
  float max_distance  = glm::length( limit.position - start );
  AABB box = parent->aabb;
  if( HitBoundingBox( box, start, dir, hit ) ){
    float hit_distance = glm::length( hit - start );
    if( hit_distance <= max_distance ){
      if( parent -> leaf ){
        PhotonSeg segments[2];
        segments[0] = parent->segments[0];
        segments[1] = parent->segments[1];
        for( unsigned int i = 0; i < sizeof(segments)/sizeof(segments[0]); i++ ){
          PhotonSeg seg = segments[i];
          if( seg.id != -1 ){
            CylIntersection intersect;
            if( HitCylinder( start, dir, seg, intersect ) ){
              if( intersect.valid ){
                double _int     = Integral_721( seg,
                                               intersect,
                                               extinction_c,
                                               dir );

                double phase_f  = 1 / ( 4 * PI );
                double rad      = scattering_c / ( PI * pow( seg.radius, 2 ) );

                PhotonBeam beam;
                if( seg.scattered ){
                  beam = beams_scatt[ seg.id ];
                } else {
                  beam = beams_const[ seg.id ];
                }
                current        += beam.energy * phase_f * rad * _int;
              }
            }
          }
        }
      }
      if( parent->left != NULL ){
        BeamRadianceStandard( screen, start, dir, limit, parent->left, current, beams_const, beams_scatt, row );
      }
      if( parent->right != NULL ){
        BeamRadianceStandard( screen, start, dir, limit, parent->right, current, beams_const, beams_scatt, row );
      }
    }
  }
}

 void BeamRadiance( screen* screen, vec4 start, vec4 dir, const Intersection& limit, Node* parent,
                    glm::dvec3& current, vector<PhotonBeam>& beams_const, vector<PhotonBeam> beams_scatt, int row ){
   vec4 hit;
   float max_distance  = glm::length( limit.position - start );
   AABB box = parent->aabb;
   if( HitBoundingBox( box, start, dir, hit ) ){
     // PositionShader( screen, hit, vec3( 1, 0, 1 ) );
     float hit_distance = glm::length( hit - start );
     if( hit_distance <= max_distance ){
       if( parent -> leaf ){
         PhotonSeg segments[2];
         segments[0] = parent->segments[0];
         segments[1] = parent->segments[1];
         for( unsigned int i = 0; i < sizeof(segments)/sizeof(segments[0]); i++ ){
           PhotonSeg seg = segments[i];
           if( seg.id != -1 ){
             CylIntersection intersect;
             if( seg.ada_width ){
               if( HitCone( start, dir, seg, intersect ) ){
                 if( intersect.valid ){

                   double _int     = Integral_721_ada( seg,
                                                      intersect,
                                                      extinction_c,
                                                      dir );

                   double phase_f  = 1 / ( 4 * PI );

                   PhotonBeam beam;
                   if( seg.scattered ){
                     beam = beams_scatt[ seg.id ];
                   } else {
                     beam = beams_const[ seg.id ];
                   }
                   current        += beam.energy * phase_f * _int;
                 }
               }
             } else if( HitCylinder( start, dir, seg, intersect ) ){
               if( intersect.valid ){
                 if( ONE_DIMENSIONAL ){
                   double _int     = Integral_73( seg,
                                                 intersect,
                                                 extinction_c,
                                                 dir );

                   double phase_f  = 1 / ( 4 * PI );
                   PhotonBeam beam;
                   if( seg.scattered ){
                     beam = beams_scatt[ seg.id ];
                   } else {
                     beam = beams_const[ seg.id ];
                   }

                   current        += beam.energy * phase_f * _int;
                 } else {
                   double _int     = Integral_721( seg,
                                                  intersect,
                                                  extinction_c,
                                                  dir );

                   double phase_f  = 1 / ( 4 * PI );
                   double rad      = scattering_c / ( PI * pow( seg.radius, 2 ) );

                   PhotonBeam beam;
                   if( seg.scattered ){
                     beam = beams_scatt[ seg.id ];
                   } else {
                     beam = beams_const[ seg.id ];
                   }
                   current        += beam.energy * phase_f * rad * _int;
                 }
               }
             }
           }
         }
       }
       if( parent->left != NULL ){
         BeamRadiance( screen, start, dir, limit, parent->left, current, beams_const, beams_scatt, row );
       }
       if( parent->right != NULL ){
         BeamRadiance( screen, start, dir, limit, parent->right, current, beams_const, beams_scatt, row );
       }
     }
   }
 }


#endif
