#pragma once

namespace phx
{
   class IRenderer
   {
   public:
      virtual void DrawPoint2D(double size, unsigned int color, const float* p) = 0 ;
	  virtual void DrawCircle2D(float width, float radius, unsigned int color, const float* p) = 0 ;
      virtual void DrawLine2D(double width, unsigned int color, const float* p1, const float * p2) = 0 ;
      virtual void DrawText(const char* text, const float* p) = 0 ;
   } ;
} ;