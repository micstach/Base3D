#include <stdafx.h>
#include "phx\phx_constraint.h"
#include "phx\phx_simulation.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

phx::Constraints::Constraint::Constraint(phx::Simulation* simulation)
    : Base(simulation)
    , m_simulation(simulation)
{}

phx::Constraints::Constraint::~Constraint()
{
}

