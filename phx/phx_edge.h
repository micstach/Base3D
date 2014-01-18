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
   class Edge : public Base
   {
   protected:
      int m_id ;
      Simulation* m_simulation ;
     
   public:
      Edge(Simulation* simulation) ;
      virtual ~Edge() ;
      
      virtual void Serialize(IStorage* storage) ;
      virtual int GetID() const { return m_id ; }

      virtual void Initialize() = 0 ;
      virtual void Reset() = 0 ;
   } ;
} 