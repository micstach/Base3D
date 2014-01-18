#pragma once

#include "phx\phx_constraint.h"

namespace phx {
   class Simulation ;
} 

namespace phx
{
   namespace Constraints
   {
      class HalfSpace : public Constraint
      {
      protected:
         Math::TVec4<float> m_position ;
         Math::TVec4<float> m_normal ;

      public:
         HalfSpace(Simulation* simulation) ;
         virtual ~HalfSpace() ;

         void Set(const Math::TVec4<float> position, const Math::TVec4<float> normal) ;
         virtual void Resolve(int step) ;

         virtual void Render(IRenderer* renderer) ;
         virtual void Serialize(IStorage* storage) ;

         PHX_CLASS_NAME(HalfSpace) 
      } ;
   } 
} 