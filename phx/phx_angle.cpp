#include <stdafx.h>

#include "phx\phx_angle.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

phx::Constraints::Angle::Angle(phx::Simulation* simulation)
   : phx::Constraints::Constraint(simulation)
{}

phx::Constraints::Angle::~Angle()
{
}

void phx::Constraints::Angle::Render(phx::IRenderer* renderer)
{
   return ;
}

void phx::Constraints::Angle::Resolve(int step)
{
   return ;
}

void phx::Constraints::Angle::Serialize(IStorage* storage) 
{
   return ;
}
