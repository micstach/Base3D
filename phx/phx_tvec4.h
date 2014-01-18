#pragma once
#include <math.h>
#include "phx\phx_base.h"
#include "phx\phx_istorage.h"
#include "phx\phx_irenderer.h"

namespace phx
{
	namespace Math
	{
		template<class T> class TVec4 
		{
		public:
			T x, y, z, w ;

			TVec4() ;
			TVec4(const TVec4<T>& vec4) ;
			TVec4(TVec4<T>& vec4) ;
			TVec4(T x, T y, T z, T w) ;

			T Distance(const TVec4<T>& vec4) const ;
			T SqDistance(const TVec4<T>& vec4) const ;
			T Length() const ;
			T SqLength() const ;
		 
			void Zero() ;

			TVec4<T>& operator += (const TVec4<T>& vec4) ;
			TVec4<T>& Normalize() ;
			TVec4<T>& operator -= (const TVec4<T>& vec4) ;
			float operator | (const TVec4<T>& vec4) const ;
			TVec4<T>& operator *= (float s) ;
			TVec4<T> operator - (const TVec4<T>& vec4) const ;
			TVec4<T> operator + (const TVec4<T>& vec4) const ;
			TVec4<T> operator - () const ;

			bool operator == (const TVec4<T>& v) const ;

			virtual void Serialize(IStorage* storage) ;
			virtual void Render(IRenderer* renderer) ;

			static TVec4<T> Reflect(const TVec4<T>& v, const TVec4<T>& n) ;
            static TVec4<T> Project(const TVec4<T>& v, const TVec4<T>& n) ;
		} ;

      template<class T> TVec4<T>::TVec4() 
      {
         x = 0.0 ;
         y = 0.0 ;
         z = 0.0 ;
         w = 0.0 ;
      }

      template<class T> TVec4<T>::TVec4(const TVec4<T>& vec4) 
      {
         x = vec4.x ;
         y = vec4.y ;
         z = vec4.z ;
         w = vec4.w ;
      }

      template<class T> TVec4<T>::TVec4(TVec4<T>& vec4) 
      {
         x = vec4.x ;
         y = vec4.y ;
         z = vec4.z ;
         w = vec4.w ;
      }

      template<class T> TVec4<T>::TVec4(T _x, T _y, T _z, T _w) 
      {
         x = _x ;
         y = _y ;
         z = _z ;
         w = _w ;
      }

      template<class T> void TVec4<T>::Zero()
      {
         x = 0.0 ;
         y = 0.0 ;
         z = 0.0 ;
         w = 0.0 ;
      }

	template<class T> TVec4<T>& TVec4<T>::Normalize() 
	{
		float length = this->Length() ;
		if (length > 0.0f)
		{
			*this *= 1.0f / length ;
		}
		return *this ;
	}

      template<class T> T TVec4<T>::Distance(const TVec4<T>& vec4) const
      {
         T dx2 = x - vec4.x ; dx2 *= dx2 ;
         T dy2 = y - vec4.y ; dy2 *= dy2 ;
         T dz2 = z - vec4.z ; dz2 *= dz2 ;
         return sqrt(dx2 + dy2 /*+ dz2*/) ;
      }

      template<class T> T TVec4<T>::SqDistance(const TVec4<T>& vec4) const
      {
         T dx2 = x - vec4.x ; dx2 *= dx2 ;
         T dy2 = y - vec4.y ; dy2 *= dy2 ;
         T dz2 = z - vec4.z ; dz2 *= dz2 ;
         return (dx2 + dy2 /*+ dz2*/) ;
      }

	  template<class T> T TVec4<T>::Length() const
      {
         return sqrt((x*x) + (y*y) /*+ (z*z)*/) ;
      }

      template<class T> T TVec4<T>::SqLength() const
      {
         return (x*x) + (y*y) /*+ (z*z)*/ ;
      }

      template<class T> TVec4<T> TVec4<T>::operator - (const TVec4<T>& vec4) const  
      {
         return TVec4<T>(x - vec4.x, y - vec4.y, 0.0, 0.0) ;
      }

	  template<class T> bool TVec4<T>::operator == (const TVec4<T>& v) const  
      {
         return (max(x - v.x, y - v.y) < 0.001f) ;
      }

      template<class T> TVec4<T> TVec4<T>::operator + (const TVec4<T>& vec4) const  
      {
         return TVec4<T>(x + vec4.x, y + vec4.y, 0.0, 0.0) ;
      }

      template<class T> TVec4<T> TVec4<T>::operator - () const
      {
         return TVec4<T>(-x, -y, 0.0, 0.0) ;
      }

      template<class T> TVec4<T>& TVec4<T>::operator += (const TVec4<T>& vec4) 
      {
         x += vec4.x ;
         y += vec4.y ;
         // z += vec4.z ;
         return *this ; 
      }
      
      template<class T> float TVec4<T>::operator | (const TVec4<T>& vec) const
      {
         return (x * vec.x + y * vec.y) ;
      }

      template<class T> TVec4<T>& TVec4<T>::operator -= (const TVec4<T>& vec4) 
      {
         x -= vec4.x ;
         y -= vec4.y ;
         // z -= vec4.z ;
         return *this ; 
      }
      
      template<class T> TVec4<T>& TVec4<T>::operator *= (float s) 
      {
         x *= s ;
         y *= s ;
         // z *= s ;
         return *this ; 
      }

      template<class T> TVec4<T> TVec4<T>::Reflect(const TVec4<T>& v, const TVec4<T>& n)
      {
          // n must be unit length
          return ((n * (2.0f * (n | v))) - v) ;
      }

      template<class T> TVec4<T> TVec4<T>::Project(const TVec4<T>& v, const TVec4<T>& n)
      {
          // n must be unit length
          return (v - n * (v|n)) ;
      }

      template<class T> void TVec4<T>::Serialize(phx::IStorage* storage) 
      {
         if (storage->IsSource())
         {
            storage->Read(x) ;
            storage->Read(y) ;
            storage->Read(z) ;
            storage->Read(w) ;
         }
         else
         {
            storage->Write(x) ;
            storage->Write(y) ;
            storage->Write(z) ;
            storage->Write(w) ;
         }
      }

      template<class T> void TVec4<T>::Render(phx::IRenderer* renderer) 
      {
         _ASSERTE(false) ;
      }

      template<class T> TVec4<T> operator * (const TVec4<T>& v, const T s) 
      {
         return Math::TVec4<T>(v.x * s, v.y * s, v.z * s, v.w * s) ;
      }
   }
}

