#include <stdafx.h>
#include "phx\phx_particle.h"
#include "phx\phx_object.h"
#include "phx\phx_istorage.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

phx::Particle::Particle(Simulation* simulation) 
   : Base(simulation)
   , m_fixed(false) 
{}

phx::Particle::~Particle()
{
}

bool phx::Particle::IsFixed() const
{
   return m_fixed ; 
}

void phx::Particle::Serialize(phx::IStorage* storage) 
{
    phx::Base::Serialize(storage) ;

    if (storage->IsSource())
    {
        storage->Read(m_fixed) ;
    }
    else
    {
        storage->Write(m_fixed) ;
    }
}

phx::Math::TVec4<float>& phx::Particle::ForceGet() 
{
    return phx::Math::TVec4<float>(0.0f, 0.0f, 0.0f, 0.0f) ;
}