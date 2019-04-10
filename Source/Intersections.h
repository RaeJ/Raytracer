#ifndef INTERSECTIONS_H
#define INTERSECTIONS_H

// -------------------------------------------------------------------------- //
// STRUCTS

struct Intersection
{
  vec4 position;
  float distance;
  int index;
  bool water;
};

struct CylIntersection
{
  bool valid;
  float tb_minus;
  float tb_plus;
  float tc_minus;
  float tc_plus;
  vec3 entry_point;
  vec3 exit_point;
  vec3 entry_normal;
  vec3 exit_normal;
};

struct AABB
{
  vec4 max;
  vec4 min;
  vec4 mid;
};

// TODO: Should move PhotonSeg into CastPhotons.h
struct PhotonSeg
{
  vec4 start;
  vec4 end;
  vec4 mid;
  vec4 min;
  vec4 max;
  vec4 orig_start;
  float radius;
  bool ada_width;
  int id;
};

// -------------------------------------------------------------------------- //
// GLOBAL VARIABLES

float m = std::numeric_limits<float>::max();

// -------------------------------------------------------------------------- //
// FUNCTIONS

bool ClosestIntersection(
  vec4 start,
  vec4 dir,
  Intersection & closestIntersections,
  const mat4& matrix,
  const vector<Triangle>& triangles
);
bool HitCylinder( const vec4 start,
                  const vec4 dir,
                  const PhotonSeg& seg,
                  CylIntersection& intersection );
bool HitCone( const vec4 start,
              const vec4 dir,
              const PhotonSeg& seg,
              CylIntersection& intersection );
bool HitBoundingBox( AABB box, vec4 start, vec4 dir, vec4& hit );
mat3 findRotationMatrix( vec3 current_dir,
                         vec3 wanted_dir );

bool HitBoundingBox( AABB box, vec4 start, vec4 dir, vec4& hit ){
  const int DIMS = 3; int RIGHT=0; int LEFT=1; int MIDDLE=2;
  int quadrant[DIMS];
  bool inside = true;
  float min_b[DIMS] = { box.min.x, box.min.y, box.max.z };
  float max_b[DIMS] = { box.max.x, box.max.y, box.min.z };
  float candidate_plane[DIMS];
  float distances[DIMS];
  float position[DIMS];

  dir                   = glm::normalize( dir );
  float origin[DIMS]    = {start.x, start.y, start.z};
  float direction[DIMS] = {dir.x,   dir.y,   dir.z};

  for ( int i=0; i<DIMS; i++ ){
    if( origin[i] < min_b[i] ){
      quadrant[i]        = LEFT;
      candidate_plane[i] = min_b[i];
      inside             = false;
    } else if ( origin[i] > max_b[i] ){
      quadrant[i]        = RIGHT;
      candidate_plane[i] = max_b[i];
      inside             = false;
    } else {
      quadrant[i]        = MIDDLE;
    }
  }

  if( inside ){
    hit = vec4( start.x, start.y, start.z, 1.0f );
    return true;
  }

  for( int i=0; i<DIMS; i++ ){
    if(  direction[i] != 0 && quadrant[i] != MIDDLE ){
      distances[i] =  ( candidate_plane[i] - origin[i] ) / direction[i];
    } else {
      distances[i] = -1;
    }
  }

  int largest = 0;
  for( int i=0; i<DIMS; i++ ){
    if( distances[largest] < distances[i] ){
      largest = i;
    }
  }

  if( distances[largest] < 0 ) return false;
  for( int i=0; i<DIMS; i++ ){
    if ( largest != i ){
      position[i] = origin[i] + ( distances[largest] * direction[i] );
      if (position[i] < min_b[i] || position[i] > max_b[i]){

				return false;
      }
    } else {
      position[i] = candidate_plane[i];
    }
  }

  hit = vec4( position[0], position[1], position[2], 1.0f );
  return true;
}

bool HitCone( const vec4 start, const vec4 dir, const PhotonSeg& seg,
  CylIntersection& intersection ){
    bool first_inte = false;
    vec3 first_normal;
    vec3 second_normal;

    vec3 l_unit_dir = glm::normalize( vec3( dir ) );
    vec3 difference = vec3( seg.end - seg.orig_start );
    vec3 w_unit_dir = glm::normalize( vec3( 0.0f,
                                            glm::length( difference ),
                                            0.0f ) );

    float S_inv     = 1/glm::length( difference ) ;

    vec3 c_unit_dir = difference * S_inv;

    mat3 R_inv      = findRotationMatrix( c_unit_dir, w_unit_dir );

    vec3 T_inv      = -( R_inv * vec3( seg.orig_start ) * S_inv );

    vec3 origin_prime    = ( R_inv * ( vec3( start ) * S_inv  ) ) + T_inv;
    vec3 dir_prime       = R_inv * l_unit_dir;

    // vec3 apex            = ( R_inv * ( vec3( seg.orig_start ) * S_inv  ) ) + T_inv;
    float height         = glm::length( seg.end - seg.orig_start );
    float r1             = seg.radius * S_inv;
    // float r2             = r1 * glm::length( apex - vec3( seg.start ) ) /
    //                        glm::length( apex - vec3( seg.orig_start ) );
    float tan_theta      = r1 / height;

    float min            = ( ( R_inv * vec3( seg.start ) * S_inv ) + T_inv ).y;

    vec3 first_hit;
    vec3 second_hit;

    // Quadratic Formula
    float t0 = -1, t1 = -1;

    // a=xD2+yD2, b=2xExD+2yEyD, and c=xE2+yE2-1.
    float a = pow( cos( tan_theta ), 2)
            * ( dir_prime[0] * dir_prime[0]
            + dir_prime[2] * dir_prime[2] )
            - pow( sin( tan_theta ), 2)
            * dir_prime[1] * dir_prime[1];

    float b = pow( cos( tan_theta ), 2)
            * ( 2 * origin_prime[0] * dir_prime[0]
            + 2 * origin_prime[2] * dir_prime[2] )
            - pow( sin( tan_theta ), 2)
            * 2 * origin_prime[1] * dir_prime[1];

    float c = pow( cos( tan_theta ), 2)
            * ( origin_prime[0] * origin_prime[0]
            + origin_prime[2] * origin_prime[2] )
            - pow( sin( tan_theta ), 2)
            * origin_prime[1] * origin_prime[1];

    float b24ac = b*b - 4*a*c;

    if( b24ac < 0 ) return false;

    float sqb24ac = sqrtf(b24ac);
    t0 = (-b + sqb24ac) / (2 * a);
    t1 = (-b - sqb24ac) / (2 * a);

    if (t0>t1) {float tmp = t0;t0=t1;t1=tmp;}

    float y0 = origin_prime[1] + t0 * dir_prime[1];
    float y1 = origin_prime[1] + t1 * dir_prime[1];
    if ( y0 < min )
    {
      if( y1 < min ) return false;
      else
      {
        // hit the cap
        float th1 = t0 + (t1-t0) * (y0-min) / (y0-y1);
        if ( th1 <= 0 ) return false;
        first_hit = origin_prime + ( dir_prime * th1 );
        first_inte = true;
        first_normal = vec3(0, -1, 0);

        if( y1 <= 1 ){
          second_hit = origin_prime + ( dir_prime * t1 );
          second_normal = glm::normalize( vec3( second_hit.x, 0, second_hit.z) );
        } else {
          float th2 = t1 + (t1-t0) * (y1-1) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
            second_normal = vec3(0, -1, 0);
          }
        }
      }
    }
    else if ( y0 >= min && y0 <= 1 )
    {
      // hit the cylinder bit
      if( t0 <= 0 ) return false;
      first_hit = origin_prime + ( dir_prime * t0 );
      first_inte = true;
      first_normal = glm::normalize( vec3( first_hit.x, 0, first_hit.z) );

      if( y1 >= min ){
        if( y1 <= 1 ){
          second_hit = origin_prime + ( dir_prime * t1 );
          second_normal = glm::normalize( vec3( -second_hit.x, 0, -second_hit.z) );
        } else {
          float th2 = t1 + (t1-t0) * (y1-1) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
            second_normal = vec3(0, -1, 0);
          }
        }
      } else {
        float th2 = t0 + (t1-t0) * (y0-min) / (y0-y1);
        if (th2>0){
          second_hit = origin_prime + ( dir_prime * th2 );
          second_normal = vec3(0, 1, 0);
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
        first_normal = vec3(0, 1, 0);
        if( y1 >= 0 ){
          second_hit = origin_prime + ( dir_prime * t1 );
          second_normal = glm::normalize( vec3( -second_hit.x, 0, -second_hit.z) );
        } else {
          float th2 = t0 + (t1-t0) * (y0-min) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
            second_normal = vec3(0, 1, 0);
          }
        }
      }
    }
    if( ( t0 > 0 ) && ( t1 > 0 ) && ( t1 > t0 ) ){
      intersection.valid       = true;
      // intersection.tb_minus    = ( first_hit.y - min ) / S_inv;
      // intersection.tb_plus     = ( second_hit.y - min ) / S_inv;
      // intersection.tc_minus    = t0 / S_inv;
      // intersection.tc_plus     = t1 / S_inv;
      intersection.entry_point = ( glm::inverse( R_inv ) * ( first_hit - T_inv ) ) / S_inv;
      intersection.exit_point  = ( glm::inverse( R_inv ) * ( second_hit - T_inv ) ) / S_inv;
      intersection.entry_normal= ( glm::inverse( R_inv ) * ( first_normal ) );
      intersection.exit_normal = ( glm::inverse( R_inv ) * ( second_normal ) );
      intersection.tb_minus    = glm::length( seg.start - seg.orig_start ) + ( first_hit.y / S_inv );
      intersection.tb_plus     = glm::length( seg.start - seg.orig_start ) + ( second_hit.y / S_inv );
      intersection.tc_minus    = glm::length( intersection.entry_point );
      intersection.tc_plus     = glm::length( intersection.exit_point );
    } else {
      intersection.valid       = false;
    }
    return first_inte;
}

bool HitCylinder( const vec4 start, const vec4 dir, const PhotonSeg& seg,
  CylIntersection& intersection ){
    bool first_inte = false;
    vec3 first_normal;
    vec3 second_normal;

    vec3 l_unit_dir = glm::normalize( vec3( dir ) );
    vec3 difference = vec3( seg.end - seg.start );
    vec3 w_unit_dir = glm::normalize( vec3( 0.0f,
                                            glm::length( difference ),
                                            0.0f ) );

    float S_inv     = 1/glm::length( difference ) ;

    vec3 c_unit_dir = difference * S_inv;

    mat3 R_inv      = findRotationMatrix( c_unit_dir, w_unit_dir );

    vec3 T_inv      = -( R_inv * vec3( seg.start ) * S_inv );

    vec3 origin_prime    = ( R_inv * ( vec3( start ) * S_inv  ) ) + T_inv;
    vec3 dir_prime       = R_inv * l_unit_dir;

    vec3 first_hit;
    vec3 second_hit;

    // Quadratic Formula
    float t0 = -1, t1 = -1;

    // a=xD2+yD2, b=2xExD+2yEyD, and c=xE2+yE2-1.
    float a = dir_prime[0] * dir_prime[0]
            + dir_prime[2] * dir_prime[2];

    float b = 2 * origin_prime[0] * dir_prime[0]
            + 2 * origin_prime[2] * dir_prime[2];

    // TODO: Work out which of the below is correct
    float c = origin_prime[0] * origin_prime[0]
            + origin_prime[2] * origin_prime[2]
            - pow( seg.radius * S_inv, 2 );
    // float c = origin_prime[0] * origin_prime[0]
    //         + origin_prime[2] * origin_prime[2]
    //         - pow( seg.radius, 2 );

    float b24ac = b*b - 4*a*c;


    if( b24ac < 0 ) return false;

    float sqb24ac = sqrtf(b24ac);
    t0 = (-b + sqb24ac) / (2 * a);
    t1 = (-b - sqb24ac) / (2 * a);

    if (t0>t1) {float tmp = t0;t0=t1;t1=tmp;}

    float y0 = origin_prime[1] + t0 * dir_prime[1];
    float y1 = origin_prime[1] + t1 * dir_prime[1];
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
        first_normal = vec3(0, -1, 0);

        if( y1 <= 1 ){
          second_hit = origin_prime + ( dir_prime * t1 );
          second_normal = glm::normalize( vec3( second_hit.x, 0, second_hit.z) );
        } else {
          float th2 = t1 + (t1-t0) * (y1-1) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
            second_normal = vec3(0, -1, 0);
          }
        }
      }
    }
    else if ( y0 >= 0 && y0 <= 1 )
    {
      // hit the cylinder bit
      // if( t0 <= 0 ) cout << "false" << endl;
      if( t0 <= 0 ) return false;
      first_hit = origin_prime + ( dir_prime * t0 );
      first_inte = true;
      first_normal = glm::normalize( vec3( first_hit.x, 0, first_hit.z) );

      if( y1 >= 0 ){
        if( y1 <= 1 ){
          second_hit = origin_prime + ( dir_prime * t1 );
          second_normal = glm::normalize( vec3( -second_hit.x, 0, -second_hit.z) );
        } else {
          float th2 = t1 + (t1-t0) * (y1-1) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
            second_normal = vec3(0, -1, 0);
          }
        }
      } else {
        float th2 = t0 + (t1-t0) * (y0) / (y0-y1);
        if (th2>0){
          second_hit = origin_prime + ( dir_prime * th2 );
          second_normal = vec3(0, 1, 0);
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
        first_normal = vec3(0, 1, 0);
        if( y1 >= 0 ){
          second_hit = origin_prime + ( dir_prime * t1 );
          second_normal = glm::normalize( vec3( -second_hit.x, 0, -second_hit.z) );
        } else {
          float th2 = t0 + (t1-t0) * (y0) / (y0-y1);
          if ( th2 > 0 ){
            second_hit = origin_prime + ( dir_prime * th2 );
            second_normal = vec3(0, 1, 0);
          }
        }
      }
    }
    if( ( t0 > 0 ) && ( t1 > 0 ) && ( t1 > t0 ) ){
      intersection.valid       = true;
      // intersection.tb_minus    = first_hit.y / S_inv;
      // intersection.tb_plus     = second_hit.y / S_inv;
      // intersection.tc_minus    = t0 / S_inv;
      // intersection.tc_plus     = t1 / S_inv;
      intersection.entry_point = ( glm::inverse( R_inv ) * ( first_hit - T_inv ) ) / S_inv;
      intersection.exit_point  = ( glm::inverse( R_inv ) * ( second_hit - T_inv ) ) / S_inv;
      intersection.entry_normal= ( glm::inverse( R_inv ) * ( first_normal ) );
      intersection.exit_normal = ( glm::inverse( R_inv ) * ( second_normal ) );
      intersection.tb_minus    = glm::length( seg.start - seg.orig_start ) + ( first_hit.y / S_inv );
      intersection.tb_plus     = glm::length( seg.start - seg.orig_start ) + ( second_hit.y / S_inv );
      intersection.tc_minus    = glm::length( intersection.entry_point );
      intersection.tc_plus     = glm::length( intersection.exit_point );

    } else {
      intersection.valid       = false;
    }
    return first_inte;
}

bool ClosestIntersection( vec4 start, vec4 dir,
                         Intersection &closestIntersections,
                         const mat4& matrix,
                         const vector<Triangle>& triangles ) {
  bool found = false;
  closestIntersections.distance = m;

  int vec_size        = triangles.size();
  int triangle_number = vec_size + waves.size();

  for(int i = 0; i < triangle_number; i++){
    vec4 v0, v1, v2;
    if( i < vec_size ){
      v0 = matrix * triangles[i].v0;
      v1 = matrix * triangles[i].v1;
      v2 = matrix * triangles[i].v2;
      closestIntersections.water = false;
    }
    // else {
    //   v0 = matrix * waves[i-vec_size].v0;
    //   v1 = matrix * waves[i-vec_size].v1;
    //   v2 = matrix * waves[i-vec_size].v2;
    //   closestIntersections.water = true;
    // }

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

mat3 findRotationMatrix( vec3 c_unit_dir, vec3 w_unit_dir ){
  vec3 v   = glm::cross( c_unit_dir, w_unit_dir );
  float c  = glm::dot( c_unit_dir, w_unit_dir );

  mat3 identity( 1.0f );

  mat3 v_x;
  v_x[0][0] = 0;    v_x[1][0] = -v.z;  v_x[2][0] = v.y;
  v_x[0][1] = v.z;  v_x[1][1] = 0;     v_x[2][1] = -v.x;
  v_x[0][2] = -v.y; v_x[1][2] = v.x;   v_x[2][2] = 0;

  mat3 R = identity + v_x + ( ( v_x * v_x) * ( 1 / ( 1 + c ) ) );

  return R;
}


#endif
