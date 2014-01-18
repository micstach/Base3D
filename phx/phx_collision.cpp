#include <stdafx.h>
#include "phx\phx_collision.h"
#include "phx\phx_point.h"
#include "phx\phx_pointex.h"
#include "phx\phx_distance.h"
#include "phx\phx_tmp_SegmentPointDistance.h"
#include "phx\phx_halfspace.h"
#include "phx\phx_particle.h"
#include "phx\phx_istorage.h"
#include "phx\phx_angle.h"
#include "phx\phx_math.h"
#include "phx_simulation.h" 

#include <limits>
#include <algorithm>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

void phx::Collision::ContactPointSegment::Resolve(float dt)
{
    // contact time
    float time = GetTime() ;

		// collision point on segment at time of collision (deltaTime before)
		phx::Math::Segment segment(m_a->GetPosition(time), m_b->GetPosition(time)) ;
		float coeff = 0.0f ;
                
		// find segment contact point
		phx::Math::TVec4<float> distanceVector = segment.DistanceVector(m_p->GetPosition(time), coeff) ;
                                
        phx::Math::TVec4<float> cn = -GetNormal() ;

        phx::Math::TVec4<float> p1_vel = m_p->GetVelocity() ;
        phx::Math::TVec4<float> p2_vel = m_a->GetVelocity() * (1.0f - coeff) + m_b->GetVelocity() * coeff ;

        phx::Math::TVec4<float> rel_vel = p2_vel - p1_vel ;


        float rel_vel_s = cn | rel_vel ;

		if (dt <= 0.0f) return ;

        dt = dt * (1.0f - time) ;

        if (rel_vel_s < -/*phx::Simulation::CONST_RELATIVE_VELOCITY_THRESHOLD*/ 0.0f)
        {
			float ma = m_a->GetMass() ;
            float mb = m_b->GetMass() ;
            float mass = ma * (1.0f - coeff) + mb * coeff ;
            float im1 = m_p->GetInvMass() ;
            float im2 = 1.0f / mass ;

			// tangent velocity
    //        phx::Math::TVec4<float> nvel = cn * rel_vel_s ;
    //        phx::Math::TVec4<float> tvel = rel_vel - nvel ;

    //        float tnlen = tvel.Length() ;
    //        float relvel_l = rel_vel.Length() ;

    //        phx::Math::TVec4<float> rvn = rel_vel * (1.0f / relvel_l) ;

    //        if (tnlen > 0.0f)// && rel_vel_s < -0.25f )
    //        {
    //            float j = phx::Simulation::Physics::Impulse(dt, /*CONST_FRICTION_FACTOR*/ 0.0f, tnlen, im1, im2) * 0.5f;

		  //      phx::Math::TVec4<float> tn = tvel * (1.0f / tnlen);

    //            m_p->ForceAdd(tn * j) ;
    //           


				//phx::Math::TVec4<float> cv = (tn * j) ;


    //            m_a->ForceAdd(-cv * (1.0f - coeff)) ;
    //            m_a->ForceAdd(-cv * (coeff)) ;
    //        }


            float j = phx::Simulation::Physics::Impulse(dt, /*CONST_ELASTIC_FACTOR*/ 0.0f, rel_vel_s, im1, im2) ;

            m_p->ForceAdd(cn * j ) ;


            phx::Math::TVec4<float> cv = (cn * (j)) ;

            //float cg = mb / (ma + mb) ;
            //float x = (coeff - cg) ;

            //m_a->ForceAdd(-cv * (- x)) ;
            //m_b->ForceAdd(-cv * (  x)) ;

			//float c = 0.5f - abs(x) ;
			//if (c < 0.0f) c = 0.0f ;
			//c *= 2.0f ;

			m_a->ForceAdd(-cv * (1.0f - coeff)) ; 
            m_b->ForceAdd(-cv * (coeff)) ;

            // integrate
            m_p->Integrate(dt) ;
            
            m_a->Integrate(dt) ;
            m_b->Integrate(dt) ;
            
		}
}

void phx::Collision::ContactPointPoint::Resolve(float dt)
{
    float time = GetTime() ;

    // objects velocities
    phx::Math::TVec4<float> p1_vel = m_a->GetVelocity() ;
    phx::Math::TVec4<float> p2_vel = m_b->GetVelocity() ;

    phx::Math::TVec4<float> rel_vel = p2_vel - p1_vel ;

    // contact normal (from p1 -> p2)
    phx::Math::TVec4<float> cn = GetNormal() ;

    float rel_vel_s = cn | rel_vel ;

    dt = dt * (1.0f - time) ;

    if (rel_vel_s < -/*phx::Simulation::CONST_RELATIVE_VELOCITY_THRESHOLD*/ 0.0f)
    {
        float im1 = m_a->GetInvMass() ;
        float im2 = m_b->GetInvMass() ;


	    //// tangent velocity
     //   phx::Math::TVec4<float> nvel = cn * rel_vel_s ;
     //   phx::Math::TVec4<float> tvel = rel_vel - nvel ;

     //   float tnlen = tvel.Length() ;
     //               
     //   if (tnlen > 0.0f && rel_vel_s < -0.1f)
     //   {
     //       float j = phx::Simulation::Physics::Impulse(dt, /*CONST_FRICTION_FACTOR*/ 0.0f, tnlen, im1, im2) * 0.5;

		   // phx::Math::TVec4<float> tn = tvel * (1.0f / tnlen);

		   // m_a->ForceAdd(tn * (j )) ;
		   // m_b->ForceAdd(tn * (-j)) ;
     //   }                

        float j = phx::Simulation::Physics::Impulse(dt , /*CONST_ELASTIC_FACTOR*/ 0.1f, rel_vel_s, im1, im2) ;

        m_a->ForceAdd(cn * j ) ;
        m_b->ForceAdd(cn * (-j)) ;


        // integrate
        m_a->Integrate(dt) ;
        m_b->Integrate(dt) ;
    }
}    

bool phx::Collision::ContactPointPoint::Detect(float radius)
{
    bool collision = false ;    

    float divFactor = 0.5f ;
    float time = 0.0f ;

    float deltaTime = 1.0f * divFactor ;

    phx::Math::TVec4<float> lcsPath[2] ;

    phx::Math::Point point(m_a->GetPosition()) ;

    int iterations = 0, maxIterations = 10 ;

    float ptime = time ;
    float ctime = ptime ;

    while (iterations < maxIterations && (time + deltaTime) <= 1.0f)
	{
		// particle <m_b> position in [time, time + deltaTime]
		phx::Math::Point p(m_b->GetPosition(time)) ;
		phx::Math::Point p_dT(m_b->GetPosition(time + deltaTime)) ;

		// particle trajectory (p2 in local coordinate system of p1) 
		lcsPath[0] = p.ToLcs(m_a->GetPosition(time)) ;
		lcsPath[1] = p_dT.ToLcs(m_a->GetPosition(time + deltaTime)) ;

		if (point.LcsIntersect(lcsPath, 2, radius))
		{
            collision = true ;

            // time "before" collision 
            ctime = ptime ;
            
			deltaTime = deltaTime * divFactor ;

			// next iteration
			iterations ++ ;
		}
		else
		{
            ptime = time ;

			time += deltaTime ;
		}
	}

    if (collision)
    {
        SetTime(ctime) ;
        SetNormal(m_b->GetPosition(ctime) - m_a->GetPosition(ctime)) ;
    }

    return collision ;
}

// detection of collision between point (particle) and line segment (build of two particles)
bool phx::Collision::ContactPointSegment::Detect(float radius)
{
	bool collision = false ;

	float divFactor = 0.5f;
    float time = 0.0f, ptime = time, ctime = ptime ;
	float deltaTime = 1.0f * divFactor ;
        
	phx::Math::TVec4<float> lcsPath[2] ;
    
	phx::Math::Segment segment(m_a->GetPosition(), m_b->GetPosition()) ;
    phx::Math::Segment cs(segment) ;

    float coeff = 0.0f ;
    phx::Math::TVec4<float> d = segment.DistanceVector(m_p->GetPosition(), coeff) ;
    if (coeff <= 0.0f || 1.0f <= coeff)
        return false ;
    if (d.SqLength() > 4.0f * radius * radius) 
        return false ;
    //if (segment.Distance(m_p->GetPosition()) > 2.0f * radius) return false ;

	int iterations = 0, maxIterations = 10 ;

	while (iterations < maxIterations && (time + deltaTime) <= 1.0f)
	{
		// segment at time
		phx::Math::Segment s(m_a->GetPosition(time), m_b->GetPosition(time)) ;
		phx::Math::Segment s_dT(m_a->GetPosition(time + deltaTime), m_b->GetPosition(time + deltaTime)) ;

		// particle trajectory (in local coordinate system of <s>) 
		lcsPath[0] = s.ToLcs(m_p->GetPosition(time)) ;
		lcsPath[1] = s_dT.ToLcs(m_p->GetPosition(time + deltaTime)) ;

		if (segment.LcsIntersect(lcsPath, 2, radius))
		{
            collision = true ;

            ctime = ptime ;
            cs = s ;
            deltaTime = deltaTime * divFactor ;

			// next iteration
			iterations ++ ;
		}
		else
		{
            ptime = time ;
			time += deltaTime ;
		}
	}

    if (collision)
    {
        SetTime(ctime) ;
        SetNormal(cs.DistanceVector(m_p->GetPosition(ctime))) ;
    }
    return collision ;
}
