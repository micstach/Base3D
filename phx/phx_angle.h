#pragma once

#include "phx\phx_constraint.h"

namespace phx
{
   namespace Constraints
   {
      class Angle : public Constraint
      {
      protected:
         float m_angle ;


      public:
         Angle(Simulation* simulation) ;
         virtual ~Angle() ;

         //virtual void Set(

         virtual void Resolve(int step) ;
         virtual float Satisfied() { return 0 ; }
         virtual void Render(IRenderer* renderer) ;
         virtual void Serialize(IStorage* storage) ;

         PHX_CLASS_NAME(Angle) 
      } ;
   } ;
}

