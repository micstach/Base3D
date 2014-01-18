#pragma once
#include <stdafx.h>
#include "phx\phx_edge.h"
#include "phx\phx_object.h"
#include "phx\phx_istorage.h"

phx::Edge::Edge(Simulation* simulation) 
    : Base(simulation)
{
}

phx::Edge::~Edge() 
{
}
      
void phx::Edge::Serialize(phx::IStorage* storage) 
{
}

void phx::Edge::Initialize()
{
}

void phx::Edge::Reset() 
{
}

