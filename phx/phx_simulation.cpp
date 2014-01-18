#include <stdafx.h>
#include "phx\phx_simulation.h"
#include "phx\phx_point.h"
#include "phx\phx_pointex.h"
#include "phx\phx_pointfixed.h"
#include "phx\phx_distance.h"
#include "phx\phx_tmp_SegmentPointDistance.h"
#include "phx\phx_halfspace.h"
#include "phx\phx_particle.h"
#include "phx\phx_istorage.h"
#include "phx\phx_angle.h"
#include "phx\phx_math.h"
#include "phx\phx_collision.h"

#include <limits>
#include <algorithm>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

phx::Simulation::Simulation()
    : m_dt(0.0f)
    , m_frame(0)
    , CONST_RELATIVE_VELOCITY_THRESHOLD(0.0f)
    , CONST_ELASTIC_FACTOR(1.0f) 
    , CONST_FRICTION_FACTOR(1.0f) 
{
}

phx::Simulation::~Simulation() 
{
}

phx::Particle* phx::Simulation::FindParticle(size_t id)
{
    for (std::vector<phx::Particle*>::iterator p = m_particles.begin(); p != m_particles.end() ; p++)
    {
        if ((*p)->GetID() == id)
        {
            return (*p) ;
        }
    }
    return NULL ;
}

int phx::Simulation::GetUniqueID()
{
    // generate unique id
    int uid = 0 ;

    while (true)
    {
        bool exists = false ;
        for (std::vector<Base*>::iterator p = m_objects.begin(); p != m_objects.end() ; p++)
        {
            if ((*p)->GetID() == uid)
            {
                exists = true ;
                break ;
            }
        }

        if (!exists)
        {
            break ;
        }

        uid++ ;
    }

    return uid ;
}

void phx::Simulation::Create(eParticle type, phx::Particle** particle) 
{
    switch(type)
    {
    case E_POINT:
        *particle = new phx::Point(this) ;
        break ;
    case E_POINT_EX:
        *particle = new phx::PointEx(this) ;
        break ;
    case E_POINT_FIXED:
        *particle = new phx::PointFixed(this) ;
        break ;
    default:
        _ASSERTE(false) ;
        *particle = NULL ;
    }

    if (*particle != NULL)
    {
        m_particles.push_back(*particle) ;
        m_objects.push_back(*particle) ;
    }
}

phx::Constraints::Constraint* phx::Simulation::Create(const char* type) 
{
   phx::Constraints::Constraint* constraint = NULL ;
   Create(type, &constraint) ;
   return constraint ;
}

void phx::Simulation::Create(const char* type, Constraints::Constraint** constraint) 
{
   if (CString(type).Compare(phx::Constraints::Distance::ClassName()) == 0)
   {
      *constraint = new phx::Constraints::Distance(this) ;
   }
   else if (CString(type).Compare(phx::Constraints::Angle::ClassName()) == 0)
   {
      *constraint = new phx::Constraints::Angle(this) ;
   }
   else if (CString(type).Compare(phx::Constraints::HalfSpace::ClassName()) == 0)
   {
      *constraint = new phx::Constraints::HalfSpace(this) ;
   }
   else
   {
      _ASSERTE(false) ;
      *constraint = NULL ;
   }

   if (*constraint != NULL)
   {
      m_constraints.push_back(*constraint) ;
      m_objects.push_back(*constraint) ;
   }
}

void phx::Simulation::SetDt(float dt)
{
    m_dt = ((dt != 0.0f) ? 0.00005f : 0.0f) ;
}

void phx::Simulation::ResolveConstraints()
{

	std::vector<phx::Constraints::Constraint*> dynamicConstraints ;
	std::vector<phx::Collision::Contact*> _inCollision ; 

    float radius = 20.0f ;

    for (int i=0; i<m_particles.size(); i++)
    {
        phx::Particle* p = m_particles[i] ;
         
        for (int k=0; k<m_constraints.size(); k++)
        {
            phx::Constraints::Distance* distance = dynamic_cast<phx::Constraints::Distance*>(m_constraints[k]) ;

            if (distance == NULL) continue ;
            if (!distance->IsCollisionObject()) continue ;
            if (p->IsFixed() && distance->GetA()->IsFixed() && distance->GetB()->IsFixed()) continue ;
            if (p == distance->GetA() || p == distance->GetB()) continue ;
            if (dynamic_cast<phx::PointEx*>(p) != NULL) continue ;

            phx::Particle* a = distance->GetA() ;
            phx::Particle* b = distance->GetB() ;

            phx::Collision::ContactPointSegment contact(a, b, p) ;

            if (contact.Detect(radius))
            {
                // contact time
                float time = contact.GetTime() ;

				// collision point on segment at time of collision (deltaTime before)
				phx::Math::Segment segment(a->GetPosition(time), b->GetPosition(time)) ;
				float coeff = 0.0f ;
                
				// find segment contact point
				phx::Math::TVec4<float> distanceVector = segment.DistanceVector(p->GetPosition(time), coeff) ;
                                
                if (coeff <= 0.0f || 1.0f <= coeff) 
                {
                    // this is ball-ball collision
                    continue ;
                }

			    // resolve at collision
                p->DeltaAdd((p->GetPosition(time) - p->GetPosition()), phx::Particle::E_MOVE_CURRENT_AND_RESET_PREVIOUS) ;
			    p->DeltaAdd(p->GetVelocity()) ;

			    a->DeltaAdd(a->GetPosition(time) - a->GetPosition(), phx::Particle::E_MOVE_CURRENT_AND_RESET_PREVIOUS) ;
                a->DeltaAdd(a->GetVelocity()) ;

			    b->DeltaAdd(b->GetPosition(time) - b->GetPosition(), phx::Particle::E_MOVE_CURRENT_AND_RESET_PREVIOUS) ;
                b->DeltaAdd(b->GetVelocity()) ;

				// contact constraint
				distanceVector.Normalize() ;
				float orientation = segment.ToLcs(p->GetPosition(time)).y > 0.0 ? 1.0f : -1.0f ;
				dynamicConstraints.push_back(new phx::Constraints::tmpSegmentPointDistance(this, p, a, b, orientation, radius));

                phx::Math::TVec4<float> cn = -contact.GetNormal() ;
                phx::Math::TVec4<float> p1_vel = p->GetVelocity() ;
                phx::Math::TVec4<float> p2_vel = a->GetVelocity() * (1.0f - coeff) + b->GetVelocity() * coeff ;
                phx::Math::TVec4<float> rel_vel = p2_vel - p1_vel ;

                float rel_vel_s = cn | rel_vel ;

				if (m_dt <= 0.0f) continue ;

				if (rel_vel_s < 0.0f)
				{
                    phx::Collision::Contact* c = new phx::Collision::ContactPointSegment(contact) ;
                    
					_inCollision.push_back(c) ;
				}
            }
        }
    }
    
    for (int p=0; p<m_particles.size(); p++)
    {
        phx::Particle* p1 = m_particles[p] ;

        for (int r=p+1; r<m_particles.size(); r++)
        {
            phx::Particle* p2 = m_particles[r] ;

            if (IsConnected(p1->GetID(), p2->GetID())) continue ;
            if (p1->IsFixed() && p2->IsFixed()) continue ;
            if (p1->GetPosition().SqDistance(p2->GetPosition()) > 2.0f * radius * radius) continue; 

            phx::Collision::ContactPointPoint contact(p1, p2) ;
            
            if (contact.Detect(radius))
            {
                float time = contact.GetTime() ;

                // objects velocities
                phx::Math::TVec4<float> p1_vel = p1->GetVelocity() ;
                phx::Math::TVec4<float> p2_vel = p2->GetVelocity() ;

                phx::Math::TVec4<float> rel_vel = p2_vel - p1_vel ;

                // contact normal (from p1 -> p2)
                phx::Math::TVec4<float> cn = contact.GetNormal() ;

                p1->DeltaAdd((p1->GetPosition(time) - p1->GetPosition()), phx::Particle::E_MOVE_CURRENT_AND_RESET_PREVIOUS) ;
                p1->DeltaAdd(p1->GetVelocity()) ;
                p2->DeltaAdd((p2->GetPosition(time) - p2->GetPosition()), phx::Particle::E_MOVE_CURRENT_AND_RESET_PREVIOUS) ;
                p2->DeltaAdd(p2->GetVelocity()) ;

                dynamicConstraints.push_back(new phx::Constraints::PushPoints(this, p1, p2, radius));

                float rel_vel_s = cn | rel_vel ;

                if (m_dt <= 0.0f) continue ;
                if (rel_vel_s < -CONST_RELATIVE_VELOCITY_THRESHOLD)
                {
                    phx::Collision::Contact* c = new phx::Collision::ContactPointPoint(contact) ;
					
                    _inCollision.push_back(c) ;
                }
            }
        }
    }


	//// apply constrains
	m_constraintsResolveSteps = 12;
    
    for (int i=0; i<m_constraintsResolveSteps; i++)
    {
		// resolve dynamic constraints
		for (int k=0; k<dynamicConstraints.size(); k++)
		{
			dynamicConstraints[k]->Resolve(i) ;
		}	
    }


	for (size_t i=0; i<_inCollision.size(); i++)
	{
		phx::Collision::Contact* collision = _inCollision[i] ;

        collision->Resolve(m_dt) ;

		// clean up
		delete _inCollision[i] ;
		_inCollision[i] = NULL ;
	}

	for (int i=0; i<m_constraintsResolveSteps; i++)
	{
		// resolve static constraints
		for (int k=0; k<m_constraints.size(); k++)
		{
			m_constraints[k]->Resolve(i) ;
		}
    }

    // (tmp) apply gravity
    if (m_dt != 0.0f)
    {
        m_frame ++ ;

        //if (_inCollision.size() == 0)

        for (size_t p=0; p<m_particles.size(); p++)
        {
            m_particles[p]->ForceAdd(Math::TVec4<float>(0.0f, -0.981f / 1000.0f, 0.0f, 0.0f) * (m_particles[p]->GetMass() / (m_dt * m_dt * 0.5f))) ;
            m_particles[p]->Integrate(m_dt) ;
        }
    }
    else
    {
    }

	// clear dynamic constraints
	for (int i=0; i<dynamicConstraints.size(); i++)
	{
		delete dynamicConstraints[i] ;
	}
	dynamicConstraints.clear() ;
}

float phx::Simulation::Physics::Impulse(float dt, float e, float relVel, float invMassA, float invMassB) 
{
    return ((1.0f + e) / (dt * dt * 0.5f)) * (relVel / (invMassA + invMassB)) ;
}

void phx::Simulation::Render(IRenderer* renderer)
{
   // draw constrains
   for (size_t c=0; c<m_constraints.size(); c++)
   {
      m_constraints[c]->Render(renderer) ;
   }

   // draw particles
   for (size_t p=0; p<m_particles.size(); p++)
   {
      m_particles[p]->Render(renderer) ;
   }
}

void phx::Simulation::Serialize(IStorage* storage) 
{
   // draw particles
   if (storage->IsSource())
   {
      int particlesCount = 0 ;
      int constraintCount = 0 ;

      storage->Read(particlesCount) ;
      for (int i=0; i<particlesCount; i++)
      {
         int type = -1 ;
         storage->Read(type) ;
         Particle* p = NULL ;
         Create((Simulation::eParticle) type, &p) ;
         p->Serialize(storage) ;
      }

      storage->Read(constraintCount) ;
      for (int i=0; i<constraintCount; i++)
      {
         char* type = NULL ;
         storage->Read(type) ;
         Constraints::Constraint* c = NULL ;
         Create(type, &c) ;
         c->Serialize(storage) ;
         delete type ;
      }
   }
    else
    {
        // store point(s)
        storage->Write((int) m_particles.size()) ;
        for (size_t p=0; p<m_particles.size(); p++)
        {
            if (dynamic_cast<phx::Point*>(m_particles[p]) != NULL &&
                dynamic_cast<phx::PointFixed*>(m_particles[p]) == NULL)
            {
                storage->Write((int) Simulation::E_POINT) ;
                m_particles[p]->Serialize(storage) ;
            }
        }

        // store pointex(s) - metapoint
        for (size_t p=0; p<m_particles.size(); p++)
        {
            if (dynamic_cast<phx::PointFixed*>(m_particles[p]) != NULL)
            {
                storage->Write((int) Simulation::E_POINT_FIXED) ;
                m_particles[p]->Serialize(storage) ;
            }
        }

        // store pointex(s) - metapoint
        for (size_t p=0; p<m_particles.size(); p++)
        {
            if (dynamic_cast<phx::PointEx*>(m_particles[p]) != NULL)
            {
                storage->Write((int) Simulation::E_POINT_EX) ;
                m_particles[p]->Serialize(storage) ;
            }
        }
        
        // store constraints
        storage->Write((int) m_constraints.size()) ;
        for (size_t c=0; c<m_constraints.size(); c++)
        {
            storage->Write(m_constraints[c]->GetClassName()) ;
            m_constraints[c]->Serialize(storage) ;
        }
    }

}
      
void phx::Simulation::Initialize()
{
   for (size_t p=0; p<m_particles.size(); p++)
   {
      m_particles[p]->Initialize() ;
   }
}

void phx::Simulation::Reset()
{
   for (size_t p=0; p<m_particles.size(); p++)
   {
      m_particles[p]->Reset() ;
   }

   m_frame = 0 ;
}

phx::Particle* phx::Simulation::FindParticle(phx::Math::TVec4<float>& point) 
{
   // find closest particle 
   phx::Particle* particle = NULL ;
   float distance = 10.0f ; // (std::numeric_limits<float>::max)() ;

   for (size_t p=0; p<m_particles.size(); p++)
   {
      float d = m_particles[p]->GetPosition().Distance(point) ;

      if (d < distance)
      {
         distance = d ;
         particle = m_particles[p] ;
      }
   }

   return particle ;
}

phx::Particle* phx::Simulation::FindParticle(const phx::Math::Ray& ray) 
{
   return NULL ;
}

phx::Base* phx::Simulation::Find(phx::Math::TVec4<float>& point) 
{
   phx::Base* object = NULL ;
   {
      float distance = (std::numeric_limits<float>::max)() ;

      for (size_t c=0; c<m_constraints.size(); c++)
      {
         float d = m_constraints[c]->DistanceTo(point) ;

         if (d < distance)
         {
            distance = d ;
            object = m_constraints[c] ;
         }
      }
   
      for (size_t p=0; p<m_particles.size(); p++)
      {
         float d = m_particles[p]->GetPosition().Distance(point) ;

         if (d < distance)
         {
            distance = d ;
            object = m_particles[p] ;
         }
      }

      if (distance > 5.0)
      {
         object = NULL ;
      }
   }
   return object ;
}

std::vector<phx::Base*>& phx::Simulation::Find(phx::Math::TVec4<float>& a, phx::Math::TVec4<float>& b) 
{
    phx::Math::Box box(a, b) ;

    m_found.clear() ;
    
    for (size_t c=0; c<m_constraints.size(); c++)
    {
        phx::Constraints::Distance* d = dynamic_cast<phx::Constraints::Distance*> (m_constraints[c]) ;

        if (d != NULL)
        {
            if (box.IsPointIn(d->GetA()->GetPosition()) || box.IsPointIn(d->GetB()->GetPosition()))
                m_found.push_back(d) ;
        }
    }
   
    for (size_t p=0; p<m_particles.size(); p++)
    {
        if (box.IsPointIn(m_particles[p]->GetPosition()))
        {
            m_found.push_back(m_particles[p]);
        }
    }

    return m_found ;
}


void phx::Simulation::Find(phx::Math::TVec4<float>& point, phx::Constraints::Constraint** constraint) 
{
   *constraint = NULL ;

   float distance = (std::numeric_limits<float>::max)() ;

   for (size_t c=0; c<m_constraints.size(); c++)
   {
      float d = m_constraints[c]->DistanceTo(point) ;

      if (d < distance)
      {
         distance = d ;
         *constraint = m_constraints[c] ;
      }
   }
}

void phx::Simulation::Delete(std::set<Base*>& objects)
{
    for (std::set<phx::Base*>::iterator i=objects.begin(); i!=objects.end(); i++)
    {
        phx::Base* p = (*i) ;
        Delete(&p) ;
    }
}

void phx::Simulation::Delete(phx::Base** object) 
{
    for (size_t c=0; c<m_constraints.size(); c++)
    {
        if (*object == m_constraints[c])
        {
            // remove object from objects
            for (size_t i=0; i<m_objects.size(); i++)
            {
                if (*object == m_objects[i])
                {
                    m_objects.erase(m_objects.begin() + i) ;
                }
            }
            
            delete m_constraints[c] ;
            m_constraints.erase(m_constraints.begin() + c) ;
            *object = NULL ;

            return ;
        }
    }

    for (size_t p=0; p<m_particles.size(); p++)
    {
        if (*object == m_particles[p])
        {
            // find constraints linked to particle
            for (size_t c=0; c<m_constraints.size(); )
            {
                if (m_constraints[c]->IsReleated(*object))
                {
                    // remove object from objects
                    for (size_t i=0; i<m_objects.size(); i++)
                    {
                        if (m_constraints[c] == m_objects[i])
                        {
                            m_objects.erase(m_objects.begin() + i) ;
                        }
                    }

                    delete m_constraints[c] ;
                    m_constraints.erase(m_constraints.begin() + c) ;
                }
                else
                {
                    c ++ ;
                }
            }

            // remove object from objects
            for (size_t i=0; i<m_objects.size(); i++)
            {
                if (*object == m_objects[i])
                {
                    m_objects.erase(m_objects.begin() + i) ;
                }
            }

            delete m_particles[p] ;
            m_particles.erase(m_particles.begin() + p) ;
            *object = NULL ;
            return ;
        }
    }

}

void phx::Simulation::SetConnected(int uidA, int uidB, bool state)
{
    if (uidA > uidB)
    {
        int c = uidA ;
        uidA = uidB;
        uidB = c ;
    }

    unsigned long key = 10000 * uidA + uidB ;
    std::map<unsigned long, bool>::iterator i = m_connected.find(key) ;
    if (i == m_connected.end())
    {
        m_connected.insert(std::map<unsigned long, bool>::value_type(key, state)) ;
    }
    else
    {
        i->second = state ;
    }
}

bool phx::Simulation::IsConnected(int uidA, int uidB) 
{
    if (uidA > uidB)
    {
        int c = uidA ;
        uidA = uidB;
        uidB = c ;
    }

    unsigned long key = 10000 * uidA + uidB ;
    std::map<unsigned long, bool>::iterator i = m_connected.find(key) ;
    if (i != m_connected.end())
    {
        return i->second ;
    }
    return false ;
}
