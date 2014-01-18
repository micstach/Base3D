#include <stdafx.h>

#include "phx_point.h"
#include "phx_irenderer.h"
#include "phx_istorage.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

phx::Point::Point(Simulation* simulation)
   : Particle(simulation)
   , m_position(Math::TVec4<float>(0.0, 0.0, 0.0, 0.0))
   , m_prevPosition(Math::TVec4<float>(0.0, 0.0, 0.0, 0.0))
   , m_backupPosition(Math::TVec4<float>(0.0, 0.0, 0.0, 0.0)) 
   , m_backupPrevPosition(Math::TVec4<float>(0.0, 0.0, 0.0, 0.0)) 
   , m_force(Math::TVec4<float>(0.0, 0.0, 0.0, 0.0)) 
   , m_velocity(Math::TVec4<float>(0.0, 0.0, 0.0, 0.0))
   , m_acceleration(Math::TVec4<float>(0.0, 0.0, 0.0, 0.0))
   , m_mass(1.0f) 
   , m_damping(1.0f)
{}

phx::Point::Point(Simulation* simulation, const Math::TVec4<float>& position) 
   : Particle(simulation)
   , m_position(position)
   , m_prevPosition(position)
   , m_backupPosition(position) 
   , m_backupPrevPosition(position) 
   , m_force(Math::TVec4<float>(0.0, 0.0, 0.0, 0.0)) 
   , m_velocity(Math::TVec4<float>(0.0, 0.0, 0.0, 0.0))
   , m_acceleration(Math::TVec4<float>(0.0, 0.0, 0.0, 0.0))
   , m_mass(1.0f) 
   , m_damping(1.0f)
{}

phx::Point::~Point()
{}

const phx::Math::TVec4<float> phx::Point::GetPosition(float delta) const 
{
    return ((m_position - m_prevPosition) * delta) + m_prevPosition ;
}

const phx::Math::TVec4<float>& phx::Point::GetPosition()
{ 
   return m_position ; 
}

const phx::Math::TVec4<float>& phx::Point::GetPrevPosition() 
{
	return m_prevPosition ;
}

phx::Math::TVec4<float>& phx::Point::GetVelocity()
{
    return m_velocity ;
}

void phx::Point::SetPosition(const Math::TVec4<float>& position) 
{
   m_position = position ; 
   m_prevPosition = position ;
   
   m_backupPosition = position ;
   m_backupPrevPosition = position ;
}

void phx::Point::DeltaAdd(const Math::TVec4<float>& delta, phx::Particle::eDeltaAddOperator op) 
{
    m_position += delta ;

	switch(op)
	{
    case phx::Particle::E_MOVE_CURRENT_AND_PREVIOUS:
		m_prevPosition += delta ;
		break ;
    case phx::Particle::E_MOVE_CURRENT_AND_RESET_PREVIOUS_AND_FIXED:
	case phx::Particle::E_MOVE_CURRENT_AND_RESET_PREVIOUS:
	    m_prevPosition = m_position ;
		break ;
	case phx::Particle::E_MOVE_CURRENT_ONLY:
		break ;
	}
}

void phx::Point::Render(IRenderer* renderer) 
{
    float size = IsFocus() ? 6.0f : 3.0f ;
   
    renderer->DrawCircle2D(IsFocus()? 3.0f : 1.0f, 10.0f, RGB(128, 128, 255), &m_position.x) ;

    if (m_mass > 1.0f)
    {
        renderer->DrawPoint2D(size, RGB(0, 0, 255), &m_position.x) ;
    }
    else
    {
        renderer->DrawPoint2D(size, RGB(0, 255, 0), &m_position.x) ;
    }

    // velocity
    phx::Math::TVec4<float> p1 = m_position ;
    phx::Math::TVec4<float> p2 = m_position + (m_prevPosition - m_position) * 10.0f ;
    renderer->DrawLine2D(1.0f, RGB(255, 255, 255), &p1.x, &p2.x) ;

    if (IsFocus())
    {
        char* text = new char [256] ;
        sprintf(text, "%d", GetID()) ;
        renderer->DrawText(text, &m_position.x) ;
    }
}

void phx::Point::Integrate(float dt)
{
    m_acceleration = m_force * (GetInvMass()) ;
    
    phx::Math::TVec4<float> x = m_acceleration * (dt * dt * 0.5f) ;

    m_velocity = (m_position - m_prevPosition) ;
   
    // debug
    float v0 = m_velocity.Length() ;

    m_prevPosition = m_position ;
    
    m_position += m_velocity  +  x ;

    m_velocity = (m_position - m_prevPosition) ;
    
    // debug
    float v1 = m_velocity.Length() ;

    // zero forces
    ForceZero() ;
}

void phx::Point::ForceZero() 
{
    m_force.Zero() ;
}

void phx::Point::ForceAdd(Math::TVec4<float>& force) 
{
    m_force += force ;
}

phx::Math::TVec4<float>& phx::Point::ForceGet()
{ 
    return m_force ;
}

void phx::Point::Serialize(IStorage* storage) 
{
    phx::Particle::Serialize(storage) ;

    if (storage->IsSource())
    {
        storage->Read(m_mass) ;
        storage->Read(m_damping) ;

        m_position.Serialize(storage) ;
        m_prevPosition.Serialize(storage) ;
        m_force.Serialize(storage) ;
        m_velocity.Serialize(storage) ;
        m_acceleration.Serialize(storage) ;
    }
    else
    {
        storage->Write(m_mass) ;
        storage->Write(m_damping) ;

        m_position.Serialize(storage) ;
        m_prevPosition.Serialize(storage) ;
        m_force.Serialize(storage) ;
        m_velocity.Serialize(storage) ;
        m_acceleration.Serialize(storage) ;
    }
}

void phx::Point::Initialize()
{
    m_backupPosition = m_position ;
    m_backupPrevPosition = m_prevPosition ; 

    m_prevPosition = m_position ;
}

void phx::Point::Reset()
{
   m_position = m_backupPosition ;
   m_prevPosition = m_backupPrevPosition ; 

   m_force.Zero() ;
   m_velocity.Zero() ;
}