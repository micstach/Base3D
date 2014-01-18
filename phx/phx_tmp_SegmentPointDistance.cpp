#include <stdafx.h>
#include "phx_tmp_SegmentPointDistance.h"
#include "phx_math.h"
#include "phx_object.h"
#include "phx_irenderer.h"
#include "phx_istorage.h"
#include "phx_simulation.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

phx::Constraints::tmpSegmentPointDistance::tmpSegmentPointDistance(Simulation* simulation, Particle* p, Particle* a, Particle* b, float orientation, float distance) 
   : phx::Constraints::Constraint(simulation)
   , m_particle(p)
   , m_segment_a(a)
   , m_segment_b(b)
   , m_distance(distance)
   , m_satisfied(0.0f)
   , m_orientation(orientation)
{
}

phx::Constraints::tmpSegmentPointDistance::~tmpSegmentPointDistance() {}

void phx::Constraints::tmpSegmentPointDistance::Resolve(int step) 
{
	phx::Math::Segment segment(m_segment_a->GetPosition(), m_segment_b->GetPosition()) ;	
    float coeff = 0.0f ;
    phx::Math::TVec4<float> v = segment.DistanceVector(m_particle->GetPosition(), coeff) ;
    
    phx::Math::TVec4<float> p = segment.ToLcs(m_particle->GetPosition()) ;
    float orientation = (p.y > 0.0f) ? 1.0f : -1.0f;  
    float dot = m_orientation * orientation ;

    if (0.0f < coeff && coeff < 1.0f)
    {
        ;
    }
    else
        return ;

    if (dot <= 0.0f)
    {
        v = -v ;
        v += (segment.GetNormal()) * m_distance * m_orientation ;
    }
    else
    {
        dot = v.Length() ;
    }

	if (dot < m_distance)
	{
        float delta = ((m_distance - dot) / dot) ;

        if (dot < 0.0f) 
        {
            delta = 1.0f ;
        }

		// translation vector
		v *= (delta * 1.0f) ;

        float ma = m_segment_a->GetInvMass() + m_segment_b->GetInvMass() ;
        float mb = m_particle->GetInvMass();
		float oom = 1.0f / (ma + mb) ;
        m_particle->DeltaAdd(v * (mb * oom), phx::Particle::E_MOVE_CURRENT_AND_PREVIOUS) ;

        m_segment_a->DeltaAdd(v * (ma * oom * (-1.0f) * (1.0f - coeff)), phx::Particle::E_MOVE_CURRENT_AND_PREVIOUS) ;
        m_segment_b->DeltaAdd(v * (ma * oom * (-1.0f) * coeff), phx::Particle::E_MOVE_CURRENT_AND_PREVIOUS) ;
	}
}

void phx::Constraints::tmpSegmentPointDistance::Render(IRenderer* renderer) 
{
	return ;
}

void phx::Constraints::tmpSegmentPointDistance::Serialize(phx::IStorage* storage)
{
   return ;
}

bool phx::Constraints::tmpSegmentPointDistance::IsReleated(const phx::Base* object) 
{
	return false ;
}

phx::Constraints::PushPoints::PushPoints(Simulation *simulation, Particle* a, Particle* b, float distance) 
    : phx::Constraints::Constraint(simulation)
    , m_a(a)
    , m_b(b) 
    , m_distance(distance)
{
}

phx::Constraints::PushPoints::~PushPoints() 
{
}

void phx::Constraints::PushPoints::Resolve(int step) 
{
    Math::TVec4<float> v = m_b->GetPosition() - m_a->GetPosition() ;

    float d = v.Length() ;   
    
    if (d < m_distance)
    {
        float delta = ((d - m_distance) / d) ;

        // translation vector
        v *= (delta) ;
		
        float ima = m_a->GetInvMass() ;
        float imb = m_b->GetInvMass() ;
		float oom = 1.0f / (ima + imb) ;

        m_a->DeltaAdd(v * (ima * oom), phx::Particle::E_MOVE_CURRENT_AND_PREVIOUS) ;
        m_b->DeltaAdd(-v * (imb * oom), phx::Particle::E_MOVE_CURRENT_AND_PREVIOUS) ;
    }
}