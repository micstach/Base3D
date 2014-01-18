#pragma once

#include <phx\phx_tvec4.h>

namespace phx
{
   namespace Math
   {
      class Const
      {
      public:
         static float pi ;
         static float eps ;
         static float infinity ;
      } ;

      class Segment
      {
      protected:
         TVec4<float> m_a ;
         TVec4<float> m_b ;
		 TVec4<float> m_d ; // segment direction vector (m_b - m_a) ;
		 
		 TVec4<float> m_x ;
		 TVec4<float> m_y ;

		 float m_sqlength ;
		 

      public:
         Segment(const TVec4<float>& a, const TVec4<float>& b) ;

         virtual ~Segment() ;

		 virtual float SqDistance(const TVec4<float>& point) ;
         virtual float Distance(const TVec4<float>& point) ;
         virtual bool Intersect(const Segment& segment) ;
		 virtual bool LcsIntersect(const TVec4<float>* path, int count, float radius);
		 virtual bool NearestPoints(const Segment& segment, TVec4<float>& p1, TVec4<float>& p2) ;

		 static bool NearestPoints(const Segment& s1, const Segment& s2, TVec4<float>& p1, TVec4<float>& p2) ;
		 const TVec4<float> DistanceVector(const TVec4<float>& point) ;
		 const TVec4<float> DistanceVector(const TVec4<float>& point, float& k) const ;
		 const TVec4<float> DistanceVector(const phx::Math::TVec4<float>& point, phx::Math::TVec4<float>& nearestPoint) const ;

		 const TVec4<float> GetNormal() const ;
		 const TVec4<float> ToLcs(const phx::Math::TVec4<float>& point) const ; 
		 const TVec4<float> GetA() const ;
		 const TVec4<float> GetB() const ;
         const TVec4<float> GetY() const { return m_y ; }
         const TVec4<float> GetPoint(float k) const ;
      } ;

      class Point
      {
      protected:
          TVec4<float> m_p ;

          TVec4<float> m_x ;
          TVec4<float> m_y ;

      public:
          Point(const TVec4<float>& p) ;

          virtual bool LcsIntersect(const TVec4<float>* path, int count, float radius);
          const TVec4<float> ToLcs(const phx::Math::TVec4<float>& point) const ; 
      } ;

      class Ray : public Segment
      {
      public:
         Ray(const Math::TVec4<float>& a, const Math::TVec4<float>& b) ;
         virtual ~Ray() ;
      } ;

      class Box
      {
      protected:
          TVec4<float> m_a ;
          TVec4<float> m_b ;

      public:
          Box(const Math::TVec4<float>& a, const Math::TVec4<float>& b) ;
          virtual ~Box() ;

          bool IsPointIn(const Math::TVec4<float>& point) ;
      } ;

   } ;
}


