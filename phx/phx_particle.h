#pragma once

#include "phx\phx_base.h"
#include "phx\phx_tvec4.h"

namespace phx {
   // core
   class Simulation ;

   // interfaces
   class IRenderer ;
   class IStorage ;
}

namespace phx
{
   class Particle : public Base
   {
   public: 
	   enum eDeltaAddOperator
	   {
		   E_MOVE_CURRENT_ONLY,
		   E_MOVE_CURRENT_AND_PREVIOUS,
		   E_MOVE_CURRENT_AND_RESET_PREVIOUS,
		   E_MOVE_CURRENT_AND_RESET_PREVIOUS_AND_FIXED
	   } ;


   protected:
      bool m_fixed ;
     
   public:
      Particle(Simulation* simulation) ;
      virtual ~Particle() ;

      virtual const Math::TVec4<float> GetPosition(float delta) const = 0 ;
      virtual const Math::TVec4<float>& GetPosition() = 0 ;
      virtual Math::TVec4<float>& GetVelocity() = 0 ;
	  virtual void SetPosition(const Math::TVec4<float>& position) = 0 ;

      // particle constrain
      virtual void DeltaAdd(const Math::TVec4<float>& delta, eDeltaAddOperator op = eDeltaAddOperator::E_MOVE_CURRENT_ONLY) = 0 ;

      // particle physics
      virtual float GetMass() = 0 ;
      virtual float GetInvMass() = 0 ;

      virtual void SetMass(float mass) = 0 ;

      virtual void Render(phx::IRenderer* renderer) = 0 ;
      virtual void Integrate(float timeStep) = 0 ;

      virtual Math::TVec4<float>& ForceGet() ;
      virtual void ForceAdd(Math::TVec4<float>& force) = 0 ;
      virtual void ForceZero() = 0 ;
     
      // (?) todo: remove is Fixed
      virtual bool IsFixed() const ;
      
      virtual void Serialize(phx::IStorage* storage) ;

      virtual void Initialize() = 0 ;
      virtual void Reset() = 0 ;
   } ;
} 