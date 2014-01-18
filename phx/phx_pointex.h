#pragma once

#include "phx\phx_particle.h"
#include <vector>

namespace phx {
   class Point ;
   class IStorage ;
   class IRenderer ;
} 

namespace phx
{
   class PointEx : public Particle
   {
   protected:
      Math::TVec4<float> m_position ;
      std::vector<std::pair<float, Particle*>> m_points ;
      
   public:

      PointEx(Simulation* simulation) ;
      virtual ~PointEx() ;
      
      virtual const Math::TVec4<float> GetPosition(float delta) const ;
      virtual const Math::TVec4<float>& GetPosition() ;
      virtual Math::TVec4<float>& GetVelocity() ;

      virtual void SetPosition(const Math::TVec4<float>& position)  ;
      virtual void AddPoint(Particle* p, float coeff) ;

      // particle constrain resolving
      virtual void DeltaAdd(const Math::TVec4<float>& delta, phx::Particle::eDeltaAddOperator op) ;

      // particle physics
      virtual float GetMass() ;
      virtual float GetInvMass() ;

      virtual void SetMass(float mass) { return ; }
      virtual void Integrate(float timeStep) { return ; }
      virtual void ForceAdd(Math::TVec4<float>& force) { return ; }
      virtual void ForceZero() { return ; }

      virtual void Initialize() {} ;
      virtual void Reset() {} ; 

      // debug rendering
      virtual void Render(IRenderer* renderer) ;
      virtual void Serialize(IStorage* storage) ;
   } ;
} 
