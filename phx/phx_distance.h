#pragma once

#include "phx\phx_constraint.h"

namespace phx {
   class Simulation ;
   class Particle ;
} ;

namespace phx
{
   namespace Constraints
   {
      class Distance : public Constraint
      {
      protected:
         float m_distance ;
         float m_rigidity ;

         bool m_collisionObject ;

         Particle* m_a ;
         Particle* m_b ;

      public:
         Distance(Simulation* simulation) ;
         Distance(Simulation* simulation, Particle* a, Particle* b, float distance = 0.0f) ;
         virtual ~Distance() ;
        
         void Set(Particle* a, Particle* b, float distance = 0.0f) ;
         void SetRigidity(float rigidity) ;
         
         void SetCollisionObject(bool state) { m_collisionObject = state ; }
         bool IsCollisionObject() { return m_collisionObject ; }

         virtual void Resolve(int step) ;
        
         Particle* GetA() { return m_a ; }
         Particle* GetB() { return m_b ; }

         virtual float Satisfied() ;
         virtual float DistanceTo(Math::TVec4<float>& point) ;
         virtual bool IsReleated(const phx::Base* object) ;

         virtual void Render(IRenderer* renderer) ;
         virtual void Serialize(IStorage* storage) ;

         PHX_CLASS_NAME(Distance) 
      } ;
   } 
}
