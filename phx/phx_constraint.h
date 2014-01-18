#pragma once

#include "phx_base.h"
#include "phx_tvec4.h"

#include <limits>

namespace phx {
   class Simulation ;
   class IRenderer ;
}

namespace phx
{
   namespace Constraints
   {
      class Constraint : public Base
      {
      protected:
         Simulation* m_simulation ;

      public:
         Constraint(Simulation* simulation) ;
         virtual ~Constraint() ;

         virtual void Resolve(int step) = 0 ;
         
         virtual float Satisfied() { return 0.0f ; }
         virtual void Render(IRenderer* p) = 0 ;
         virtual float DistanceTo(Math::TVec4<float>& point) { return (std::numeric_limits<float>::max)() ; }
         
         PHX_CLASS_NAME(Constraint) 
      } ;
   }
} 
