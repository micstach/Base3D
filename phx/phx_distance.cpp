#include <stdafx.h>
#include "phx_distance.h"
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

phx::Constraints::Distance::Distance(Simulation* simulation, Particle* a, Particle* b, float distance /* = 0 */) 
   : phx::Constraints::Constraint(simulation)
   , m_rigidity(1.0f)
   , m_collisionObject(false) 
{
   Set(a, b, distance) ;
}

phx::Constraints::Distance::Distance(Simulation* simulation) 
   : phx::Constraints::Constraint(simulation)
   , m_distance(0.0f)
   , m_rigidity(1.0f) 
   , m_collisionObject(false) 
   , m_a(NULL)
   , m_b(NULL)
{
}

phx::Constraints::Distance::~Distance()
{}

void phx::Constraints::Distance::Resolve(int step) 
{
    if (m_a->IsFixed() && m_b->IsFixed()) return ;
    
    Math::TVec4<float> v = m_b->GetPosition() - m_a->GetPosition() ;

    float d = v.Length() ;

    float delta = ((d - m_distance) / d) ;
   
    // translation vector
    if (m_rigidity < 1.0f) 
        m_rigidity = 0.25f ;

    v *= (delta * 0.5f * m_rigidity);

    float ima = m_a->GetInvMass() ;
    float imb = m_b->GetInvMass() ;
    float oom = 1.0f / (ima + imb) ;

    m_a->DeltaAdd(v * (ima * oom), phx::Particle::E_MOVE_CURRENT_AND_PREVIOUS);
    m_b->DeltaAdd(-v * (imb * oom), phx::Particle::E_MOVE_CURRENT_AND_PREVIOUS);

    if (m_simulation->GetDt() > 0.0f && step == 0)
    {
        v *= (1.0f / m_simulation->GetConstraintsResolveSteps()) ;

        m_a->ForceAdd( v * (oom / (m_simulation->GetDtSq() * 0.5f))) ;
        m_b->ForceAdd(-v * (oom / (m_simulation->GetDtSq() * 0.5f))) ;
    }
}

void phx::Constraints::Distance::Set(Particle* a, Particle* b, float distance /* = 0 */) 
{
   m_a = a ;
   m_b = b ;

   m_simulation->SetConnected(m_a->GetID(), m_b->GetID(), true) ;

   if (distance < phx::Math::Const::eps)
   {
      m_distance = m_a->GetPosition().Distance(m_b->GetPosition()) ;
   }
   else
   {
      m_distance = distance ;
   }
}

void phx::Constraints::Distance::Render(IRenderer* renderer) 
{
    float size = IsFocus() ? 4.0 : 1.0 ;

    if (m_rigidity < 1.0f)
    {
        renderer->DrawLine2D(size, RGB(255, 255, 0), &m_a->GetPosition().x, &m_b->GetPosition().x) ;
    }
    else
    {
        if (IsCollisionObject()) 
        {
            phx::Math::Segment s(m_a->GetPosition(), m_b->GetPosition()) ;

            phx::Math::TVec4<float> p1 = m_a->GetPosition() + s.GetY() * 10.0f;
            phx::Math::TVec4<float> p2 = m_b->GetPosition() + s.GetY() * 10.0f ;

            renderer->DrawLine2D(size, RGB(128, 128, 255), &p1.x, &p2.x) ;
            renderer->DrawLine2D(size * 2.0, RGB(255, 128, 128), &m_a->GetPosition().x, &m_b->GetPosition().x) ;
            
            p1 = m_a->GetPosition() - s.GetY() * 10.0f ;
            p2 = m_b->GetPosition() - s.GetY() * 10.0f ;
            renderer->DrawLine2D(size, RGB(128, 128, 255), &p1.x, &p2.x) ;
        }
        else
            renderer->DrawLine2D(size, RGB(96, 96, 96), &m_a->GetPosition().x, &m_b->GetPosition().x) ;
    }
}

float phx::Constraints::Distance::Satisfied() 
{
   Math::TVec4<float> v = m_b->GetPosition() - m_a->GetPosition() ;
   float d = v.Length() ;
   return fabs(d - m_distance) ;
}

void phx::Constraints::Distance::Serialize(phx::IStorage* storage)
{
    phx::Base::Serialize(storage) ;

    if (storage->IsSource())
    {
        storage->Read(m_distance) ;
        storage->Read(m_rigidity) ;

        int aid = -1 ;
        int bid = -1 ;

        storage->Read(aid) ;
        storage->Read(bid) ;

        m_a = m_simulation->FindParticle(aid) ;
        m_b = m_simulation->FindParticle(bid) ;

        _ASSERTE(m_a != NULL) ;
        _ASSERTE(m_b != NULL) ;

        m_simulation->SetConnected(m_a->GetID(), m_b->GetID(), true) ;

        storage->Read(m_collisionObject) ;
    }
    else
    {
        storage->Write(m_distance) ;
        storage->Write(m_rigidity) ;
        storage->Write(m_a->GetID()) ;
        storage->Write(m_b->GetID()) ;
        storage->Write(m_collisionObject) ;
    }
}

void phx::Constraints::Distance::SetRigidity(float rigidity) 
{
   m_rigidity = rigidity ;
}

float phx::Constraints::Distance::DistanceTo(phx::Math::TVec4<float>& point) 
{ 
   if (m_a->GetPosition().Distance(point) > 5.0 &&
       m_b->GetPosition().Distance(point) > 5.0)
   {
      return phx::Math::Segment(m_a->GetPosition(), m_b->GetPosition()).Distance(point) ;
   }

   return __super::DistanceTo(point) ;
}

bool phx::Constraints::Distance::IsReleated(const phx::Base* object) 
{
   return ((m_a == object) || (m_b == object)) ;
}
