#pragma once

#include "phx\phx_point.h"
#include "phx\phx_tvec4.h"

namespace phx {
   class IRenderer ;
}

namespace phx
{
   class PointFixed : public Point
   {
   public:
      PointFixed(Simulation* simulation) ;
      PointFixed(Simulation* simulation, const Math::TVec4<float>& position) ;
      virtual ~PointFixed() ;

      virtual const Math::TVec4<float> GetPosition(float delta) const ;
      virtual const Math::TVec4<float>& GetPosition() ;
      virtual const Math::TVec4<float>& GetPrevPosition() ;
      virtual void SetPosition(const Math::TVec4<float>& position) ;

      // particle constrain
      virtual void DeltaAdd(const Math::TVec4<float>& delta, phx::Particle::eDeltaAddOperator op) ;

      // particle physics
      virtual Math::TVec4<float>& GetVelocity() ;
      virtual float GetMass() ;
      virtual float GetInvMass() ;

      virtual void SetMass(float mass) { return ; }
      virtual void Integrate(float timeStep) ;
      virtual void ForceAdd(Math::TVec4<float>& force) ;
      virtual void ForceZero() ;
      virtual Math::TVec4<float>& ForceGet() ;

      virtual void Render(IRenderer* renderer) ;
      
      virtual bool IsFixed() const { return true ; }

      virtual void Initialize() ;
      virtual void Reset() ; 

      virtual void Serialize(IStorage* storage) ;
   } ;
}

