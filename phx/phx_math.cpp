#include <stdafx.h>
#include "phx_math.h"


float phx::Math::Const::pi = 3.141592f ;
float phx::Math::Const::eps = 0.0001f ;
float phx::Math::Const::infinity = 100000000000.0f ;


phx::Math::Segment::Segment(const Math::TVec4<float>& a, const Math::TVec4<float>& b) 
   : m_a(a)
   , m_b(b)
{
	m_x = m_d = m_b - m_a ;
	m_x.Normalize() ;

	m_y.x = m_x.y ;
	m_y.y = -m_x.x ;

	m_sqlength = (m_d | m_d) ;
}

phx::Math::Segment::~Segment() 
{}

const phx::Math::TVec4<float> phx::Math::Segment::GetNormal() const 
{
	return m_y ;
}

const phx::Math::TVec4<float> phx::Math::Segment::ToLcs(const phx::Math::TVec4<float>& p) const 
{
	return phx::Math::TVec4<float>((p-m_a)|m_x, (p-m_a)|m_y, 0.0f, 0.0f) ;
}

bool phx::Math::Segment::LcsIntersect(const phx::Math::TVec4<float>* path, int count, float radius)
{
	bool retval = false ;

	// (!) this segment in Lcs
	TVec4<float> a = ToLcs(m_a) ;
	TVec4<float> b = ToLcs(m_b) ;
	Segment segment(a, b) ;

	TVec4<float> segmentPoint, pathSegmentPoint ;
	
	for (int i=0; i<count-1; i++)
	{
		Segment pathSegment(path[i], path[i+1]) ;

		if (Segment::NearestPoints(segment, pathSegment, segmentPoint, pathSegmentPoint))
		{
			float dist = (pathSegmentPoint - segmentPoint).Length() ;
			retval |= (dist <= radius) ;
		}
	}

	return retval ;
}

const phx::Math::TVec4<float> phx::Math::Segment::GetA() const 
{
	return m_a ;
}

const phx::Math::TVec4<float> phx::Math::Segment::GetB() const 
{
	return m_b ;
}

const phx::Math::TVec4<float> phx::Math::Segment::GetPoint(float k) const 
{
    if (k <= 0.0f)
    {
        return m_a ;
    }
    else if (k >= 1.0f)
    {
        return m_b ;
    }
    else 
	{
        return m_a + m_d * k ;
    }
}

const phx::Math::TVec4<float> phx::Math::Segment::DistanceVector(const phx::Math::TVec4<float>& point) 
{
	float k = 0.0f ;
	return DistanceVector(point, k) ;
}

const phx::Math::TVec4<float> phx::Math::Segment::DistanceVector(const phx::Math::TVec4<float>& point, float& k) const 
{
	Math::TVec4<float> segment = m_b - m_a ;
	k = (segment | (point - m_a)) / (segment | segment) ;

	if (k <= 0.0f)
	{
		k = 0.0f ;
		return point - m_a ;
	}
	else if (k >= 1.0f)
	{
		k = 1.0f ;
		return point - m_b ;
	}
	else 
	{
		return (point - m_a) - segment * k ;
	}
}

const phx::Math::TVec4<float> phx::Math::Segment::DistanceVector(const phx::Math::TVec4<float>& point, phx::Math::TVec4<float>& nearestPoint) const 
{
	_ASSERTE(m_sqlength != 0.0f) ;

	float k = (m_d | (point - m_a)) / m_sqlength ;

	if (k <= 0.0f)
	{
		nearestPoint = m_a ;
	}
	else if (k >= 1.0f)
	{
		nearestPoint = m_b ;
	}
	else 
	{
		nearestPoint = this->GetPoint(k) ;
	}

	return (point - nearestPoint) ;
}

float phx::Math::Segment::Distance(const phx::Math::TVec4<float>& point) 
{
	return DistanceVector(point).Length() ;
}

float phx::Math::Segment::SqDistance(const TVec4<float>& point) 
{
	return DistanceVector(point).SqLength() ;
}

bool phx::Math::Segment::NearestPoints(const Segment& s1, const Segment& s2, TVec4<float>& p1, TVec4<float>& p2)
{
		phx::Math::TVec4<float> A = s1.m_b - s1.m_a ;
		phx::Math::TVec4<float> B = s2.m_b - s2.m_a ;

		float L11 =  (A | A) ;
		float L12 = -(A | B) ;
		float L22 =  (B | B) ;
		float M = L11 * L22 - L12 * L12 ;

		// no parallel case
		if (M != 0)
		{
			phx::Math::TVec4<float> BA = s2.m_a - s1.m_a ;

			float rA = (A | BA) ;
			float rB = -(B | BA) ;

			float kB = (L11*rB - L12*rA) / M ;
			float kA = (rA - (L12 * kB)) / L11 ;

			// snap coeffs to segments range
			bool _robust = false ;

			_robust = ((kA < 0.0f) || (kA > 1.0f) || (kB < 0.0f) || (kB > 1.0f)) ;

			if (_robust)
			{
				float dMin = phx::Math::Const::infinity ;
				phx::Math::TVec4<float> nearestPoint ;

				float d = 0.0f ;

				d = s1.DistanceVector(s2.GetA(), nearestPoint).SqLength() ;
				if (d <= dMin)
				{
					dMin = d ;
					p1 = nearestPoint ;
					p2 = s2.GetA() ;
				}
		
				d = s1.DistanceVector(s2.GetB(), nearestPoint).SqLength() ;
				if (d <= dMin)
				{
					dMin = d ;
					p1 = nearestPoint ;
					p2 = s2.GetB() ;
				}

				d = s2.DistanceVector(s1.GetA(), nearestPoint).SqLength() ;
				if (d <= dMin)
				{
					dMin = d ;
					p2 = nearestPoint ;
					p1 = s1.GetA() ;
				}
		
				d = s2.DistanceVector(s1.GetB(), nearestPoint).SqLength() ;
				if (d <= dMin)
				{
					dMin = d ;
					p2 = nearestPoint ;
					p1 = s1.GetB() ;
				}

				return true ;
			}
			else
			{
				p1 = s1.GetPoint(kA) ;
				p2 = s2.GetPoint(kB) ;

				return true ;	
			}
		}

		// parallel
		return false ;
}

bool phx::Math::Segment::NearestPoints(const Segment& segment, TVec4<float>& p1, TVec4<float>& p2)
{
	return Segment::NearestPoints(*this, segment, p1, p2) ;
}

bool phx::Math::Segment::Intersect(const Segment& segment) 
{
   Math::TVec4<float> A = m_b - m_a ;
   Math::TVec4<float> B = -(segment.m_b - segment.m_a) ;
   Math::TVec4<float> C = segment.m_a - m_a ;

   float d = A.x * B.y - B.x * A.y ;

   if (d < -Math::Const::eps || d >= Math::Const::eps)
   {
      float dx = C.x * B.y - B.x * C.y ;
      float x = dx / d ;

      if (0.0f <= x && x <= 1.0f)
      {
         float dy = A.x * C.y - C.x * A.y ;
         float y = dy / d ;

         if (0.0f <= y && y <= 1.0f)
         {
            return true ;
         }
      }
   }
   return false ;
}

phx::Math::Point::Point(const phx::Math::TVec4<float>& p) 
    : m_p(p)
    , m_x(1.0f, 0.0f, 0.0f, 0.0f)
    , m_y(0.0f, 1.0f, 0.0f, 0.0f)
{
}

const phx::Math::TVec4<float> phx::Math::Point::ToLcs(const phx::Math::TVec4<float>& p) const 
{
	return phx::Math::TVec4<float>((p-m_p)|m_x, (p-m_p)|m_y, 0.0f, 0.0f) ;
}

bool phx::Math::Point::LcsIntersect(const phx::Math::TVec4<float>* path, int count, float radius)
{
	bool retval = false ;

	// (!) this segment in Lcs
	TVec4<float> p = ToLcs(m_p) ;


	TVec4<float> segmentPoint, pathSegmentPoint ;
	
	for (int i=0; i<count-1; i++)
	{
		Segment pathSegment(path[i], path[i+1]) ;

        retval |= (pathSegment.Distance(p) <= radius) ;
	}

	return retval ;
}


phx::Math::Box::Box(const Math::TVec4<float>& a, const Math::TVec4<float>& b) 
{
    m_a.x = min(a.x, b.x) ;
    m_a.y = min(a.y, b.y) ;
    m_a.z = min(a.z, b.z) ;
    m_a.w = min(a.w, b.w) ;

    m_b.x = max(a.x, b.x) ;
    m_b.y = max(a.y, b.y) ;
    m_b.z = max(a.z, b.z) ;
    m_b.w = max(a.w, b.w) ;
}

phx::Math::Box::~Box() 
{
}

bool phx::Math::Box::IsPointIn(const phx::Math::TVec4<float>& point) 
{
    return (m_a.x <= point.x && point.x <= m_b.x) &&
           (m_a.y <= point.y && point.y <= m_b.y) &&
           (m_a.z <= point.z && point.z <= m_b.z) &&
           (m_a.w <= point.w && point.w <= m_b.w) ;
}
