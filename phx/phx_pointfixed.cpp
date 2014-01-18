#include <stdafx.h>

#include "phx_pointfixed.h"
#include "phx_irenderer.h"
#include "phx_istorage.h"
#include "phx_math.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

phx::PointFixed::PointFixed(Simulation* simulation)
   : Point(simulation)
{
    m_fixed = true ;
    m_mass = 0.0f ;
}

phx::PointFixed::PointFixed(Simulation* simulation, const Math::TVec4<float>& position) 
   : Point(simulation, position)
{
    m_fixed = true ;
    m_mass = 0.0f ;
}

phx::PointFixed::~PointFixed()
{}

const phx::Math::TVec4<float> phx::PointFixed::GetPosition(float delta) const 
{
    return m_position ;
}

const phx::Math::TVec4<float>& phx::PointFixed::GetPosition()
{ 
   return m_position ; 
}

const phx::Math::TVec4<float>& phx::PointFixed::GetPrevPosition() 
{
	return m_prevPosition ;
}

phx::Math::TVec4<float>& phx::PointFixed::GetVelocity()
{
   return m_velocity ;
}

void phx::PointFixed::SetPosition(const Math::TVec4<float>& position) 
{ 
   m_position = position ; 
   m_prevPosition = position ;

   m_backupPosition = position ;
   m_backupPrevPosition = position ;
}

void phx::PointFixed::DeltaAdd(const Math::TVec4<float>& delta, phx::Particle::eDeltaAddOperator op) 
{
	if (op == phx::Particle::E_MOVE_CURRENT_AND_RESET_PREVIOUS_AND_FIXED)
	{
		m_position += delta ;
		m_prevPosition = m_position ;
		m_backupPosition = m_position ;
		m_backupPrevPosition = m_prevPosition ;
	}
}

void phx::PointFixed::Render(IRenderer* renderer) 
{
    float size = IsFocus() ? 6.0f : 3.0f ;
   
    renderer->DrawCircle2D(IsFocus()? 3.0f : 1.0f, 10.0f, RGB(128, 128, 255), &m_position.x) ;
    renderer->DrawPoint2D(size, RGB(255, 0, 0), &m_position.x) ;

    if (IsFocus())
    {
        char* text = new char [256] ;
        sprintf(text, "%d", GetID()) ;
        renderer->DrawText(text, &m_position.x) ;
    }
}

void phx::PointFixed::Integrate(float timeStep)
{
    return ;
}

void phx::PointFixed::ForceZero() 
{
    return ; 
}

void phx::PointFixed::ForceAdd(Math::TVec4<float>& force) 
{
    return ;
}

float phx::PointFixed::GetMass()
{
    return phx::Math::Const::infinity ;
}

float phx::PointFixed::GetInvMass()
{
    return 0.0f ;
}

phx::Math::TVec4<float>& phx::PointFixed::ForceGet()
{ 
    return m_force ;
}

void phx::PointFixed::Serialize(IStorage* storage) 
{
    phx::Point::Serialize(storage) ;

    if (storage->IsSource())
    {
        m_position.Serialize(storage) ;
        m_prevPosition.Serialize(storage) ;
        m_force.Serialize(storage) ;
        m_velocity.Serialize(storage) ;
        m_acceleration.Serialize(storage) ;
    }
    else
    {
        m_position.Serialize(storage) ;
        m_prevPosition.Serialize(storage) ;
        m_force.Serialize(storage) ;
        m_velocity.Serialize(storage) ;
        m_acceleration.Serialize(storage) ;
    }
}

void phx::PointFixed::Initialize()
{
   m_backupPosition = m_position ;
   m_backupPrevPosition = m_prevPosition ; 
}

void phx::PointFixed::Reset()
{
   m_position = m_backupPosition ;
   m_prevPosition = m_backupPrevPosition ; 

   m_force.Zero() ;
   m_velocity.Zero() ;
}