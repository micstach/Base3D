#include <stdafx.h>
#include "phx\phx_halfspace.h"
#include "phx\phx_simulation.h"
#include "phx\phx_particle.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

phx::Constraints::HalfSpace::HalfSpace(Simulation* simulation) 
   : Constraint(simulation) 
{}

phx::Constraints::HalfSpace::~HalfSpace() 
{}

void phx::Constraints::HalfSpace::Render(IRenderer* renderer) 
{
}

void phx::Constraints::HalfSpace::Serialize(IStorage* storage) 
{
}

void phx::Constraints::HalfSpace::Set(const Math::TVec4<float> position, const Math::TVec4<float> normal) 
{
   m_position = position ;
   m_normal = normal ;
}

void phx::Constraints::HalfSpace::Resolve(int step) 
{
   for (int p=0; p<m_simulation->GetParticlesCount(); p++)
   {
      phx::Particle* particle = m_simulation->GetParticle(p) ;

      if (!particle->IsFixed())
      {
         // half space penetration depth
         float depth = ((particle->GetPosition() - m_position) | m_normal) ;

         if (depth < 0.0f)
         {
            // resolve constraint
            particle->DeltaAdd(m_normal * (-depth)) ;
         }
      }
   }
}
