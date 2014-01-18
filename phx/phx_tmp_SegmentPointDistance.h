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
      class tmpSegmentPointDistance : public Constraint
      {
      protected:
		 float m_satisfied ;
         float m_distance ;
         float m_orientation ;

         Particle* m_particle ;
		 Particle* m_segment_a ;
         Particle* m_segment_b ;

      public:
         tmpSegmentPointDistance(Simulation* simulation, Particle* p, Particle* a, Particle* b, float orientation, float distance = 0.0f) ;
         virtual ~tmpSegmentPointDistance() ;
        
         virtual void Resolve(int step) ;
         virtual float Satisfied() { return 0.0f ; }
         virtual bool IsReleated(const phx::Base* object) ;

         virtual void Render(IRenderer* renderer) ;
         virtual void Serialize(IStorage* storage) ;

         PHX_CLASS_NAME(tmpSegmentPointDistance) 
      } ;

      class PushPoints : public Constraint
      {
      protected:
          Particle* m_a ;
          Particle* m_b ;
          float m_distance ;

      public :
          PushPoints(Simulation *simulation, Particle* a, Particle* b, float distance) ;
          ~PushPoints() ;
          virtual void Resolve(int step) ;
          virtual float Satisfied() {return 0.0f ; }
          
          virtual void Render(IRenderer* renderer) { return ; }
          virtual void Serialize(IStorage* storage) { return ; }

          PHX_CLASS_NAME(PushPoints) 
      } ;
   } 
}
