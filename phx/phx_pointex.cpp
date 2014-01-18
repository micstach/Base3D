#include <stdafx.h>

#include "phx\phx_simulation.h"
#include "phx\phx_pointex.h"
#include "phx\phx_particle.h"
#include "phx\phx_irenderer.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

phx::PointEx::PointEx(Simulation* simulation)
   : phx::Particle(simulation)
{
}

phx::PointEx::~PointEx() 
{}

const phx::Math::TVec4<float>& phx::PointEx::GetPosition()
{ 
   m_position.Zero() ;
   for (std::vector<std::pair<float, Particle*>>::iterator point = m_points.begin(); point != m_points.end() ; point++)
   {
      m_position += point->second->GetPosition() * point->first ;
   }
   return m_position ; 
}

const phx::Math::TVec4<float> phx::PointEx::GetPosition(float delta) const
{
    phx::Math::TVec4<float> position ;
    for (std::vector<std::pair<float, Particle*>>::const_iterator point = m_points.begin(); point != m_points.end() ; point++)
    {
        position += point->second->GetPosition(delta) * point->first ;
    }
    return position ; 
}

phx::Math::TVec4<float>& phx::PointEx::GetVelocity() 
{ 
    phx::Math::TVec4<float> velocity ;
    for (std::vector<std::pair<float, Particle*>>::const_iterator point = m_points.begin(); point != m_points.end() ; point++)
    {
        velocity += point->second->GetVelocity() * point->first ;
    }
    return velocity ; 
}

void phx::PointEx::SetPosition(const Math::TVec4<float>& position) 
{ 
    phx::Math::TVec4<float> delta = position - GetPosition() ;

    for (std::vector<std::pair<float, Particle*>>::iterator point = m_points.begin(); point != m_points.end() ; point++)
    {
        point->second->SetPosition(point->second->GetPosition() + delta * point->first) ;
    }
}

void phx::PointEx::AddPoint(Particle* p, float coeff) 
{
   m_points.push_back(std::pair<float, Particle*>(coeff, p)) ;
}

void phx::PointEx::DeltaAdd(const Math::TVec4<float>& delta, phx::Particle::eDeltaAddOperator op) 
{
    for (std::vector<std::pair<float, Particle*>>::iterator point = m_points.begin(); point != m_points.end() ; point++)
    {
        if (!point->second->IsFixed())
        {
            point->second->DeltaAdd(delta * point->first, op) ;
        }
    }
}

float phx::PointEx::GetMass() 
{ 
    float mass = 0.0f ;
    for (std::vector<std::pair<float, Particle*>>::iterator point = m_points.begin(); point != m_points.end() ; point++)
    {
        mass += point->second->GetMass() * point->first ;
    }
    return mass ;
}

float phx::PointEx::GetInvMass() 
{ 
    float mass = 0.0f ;
    for (std::vector<std::pair<float, Particle*>>::iterator point = m_points.begin(); point != m_points.end() ; point++)
    {
        mass += point->second->GetMass() * point->first ;
    }
    return 1.0f / mass ;
}

void phx::PointEx::Render(IRenderer* renderer) 
{
   renderer->DrawCircle2D(1.0f, 10.0f, RGB(0, 255, 0), &GetPosition().x) ;
}

void phx::PointEx::Serialize(IStorage* storage) 
{
    phx::Base::Serialize(storage) ;

    if (storage->IsSource())
    {
        size_t size = 0;
        storage->Read(size) ;

        for (size_t i=0; i<size; i++)
        {
            int pid = -1 ;
            float coeff ;

            storage->Read(pid) ;
            storage->Read(coeff) ;

            AddPoint(m_simulation->FindParticle(pid), coeff) ;
        }
    }
    else
    {
        storage->Write(m_points.size()) ;

        for (size_t i=0; i<m_points.size(); i++)
        {
            // particle uid
            storage->Write(m_points[i].second->GetID()) ;

            // coefficient
            storage->Write(m_points[i].first) ;
        }
    }
}
