#pragma once

#include "phx\phx_particle.h"
#include "phx\phx_tvec4.h"

namespace phx {
   class IRenderer ;
}

namespace phx
{
   class Point : public Particle
   {
   protected:
      Math::TVec4<float> m_backupPosition ;
      Math::TVec4<float> m_backupPrevPosition ;

      Math::TVec4<float> m_position ;
      Math::TVec4<float> m_prevPosition ;
      Math::TVec4<float> m_force ;
      Math::TVec4<float> m_velocity ;
      Math::TVec4<float> m_acceleration ;
      
      float m_mass ;
      float m_damping ;

   public:
      Point(Simulation* simulation) ;
      Point(Simulation* simulation, const Math::TVec4<float>& position) ;
      virtual ~Point() ;

      virtual const Math::TVec4<float> GetPosition(float delta) const ;
      virtual const Math::TVec4<float>& GetPosition() ;
      virtual const Math::TVec4<float>& GetPrevPosition() ;
      virtual void SetPosition(const Math::TVec4<float>& position) ;

      // particle constrain
      virtual void DeltaAdd(const Math::TVec4<float>& delta, phx::Particle::eDeltaAddOperator op = phx::Particle::E_MOVE_CURRENT_ONLY) ;

      // particle physics
      virtual Math::TVec4<float>& GetVelocity() ;
      virtual float GetMass() { return m_mass ; }
      virtual float GetInvMass() { return 1.0f / m_mass ; }
      virtual void SetMass(float mass) { m_mass = mass ; }
      virtual void Integrate(float timeStep) ;
      virtual phx::Math::TVec4<float>& phx::Point::ForceGet() ;
      virtual void ForceAdd(Math::TVec4<float>& force) ;
      virtual void ForceZero() ;
      virtual bool IsFixed() const { return false ; }

      
      virtual void Render(IRenderer* renderer) ;
      
      

      virtual void Initialize() ;
      virtual void Reset() ; 

      virtual void Serialize(IStorage* storage) ;
   } ;
}

