#include <stdafx.h>
#include "phx_base.h"
#include "phx\phx_istorage.h"
#include "phx\phx_simulation.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

phx::Base::Base(Simulation* simulation)
   : m_simulation(simulation)
   , m_focus(false) 
{
    m_id = m_simulation->GetUniqueID() ;
}

phx::Base::~Base()
{
    m_simulation = NULL ;
    m_id = -1 ;
}

void phx::Base::SetFocus(bool focus)
{
   m_focus = focus ;
}

bool phx::Base::IsFocus() const 
{
   return m_focus ;
}

void phx::Base::Serialize(phx::IStorage* storage)
{
    if (storage->IsSource())
    {
        storage->Read(m_id) ;
    }
    else
    {
        storage->Write(m_id) ;
    }
}
