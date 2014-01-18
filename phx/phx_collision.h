#pragma once

#include "phx\phx_tvec4.h"
#include "phx\phx_base.h"

#include <vector>

namespace phx {

   namespace Math {
      class Ray ;
   } 

   namespace Constraints {
      class Constraint ;
   }

   class Particle ;
   class Point ;
   class PointEx ;
   class IRenderer ;
   class IStorage ;
} ;

namespace phx
{
    class Collision
    {
    public:

        class Body
        {
        public:
            Body() {} ;
            virtual ~Body() ;
        } ;

        class Point : Body
        {
        protected:
            float m_radius ;

        public:
            Point()
                : Body() 
            {
            }
            virtual ~Point() 
            {
            }
        } ;

        class Segment : Body
        {
        public:
            Segment() : Body() 
            {
            }

            virtual ~Segment()
            {
            }
        } ;

        class Triangle : Body
        {
        public:
            Triangle() : Body()
            {
            }

            virtual ~Triangle()
            {
            }
        } ;


        class Contact
        {
        protected:
            float m_time ;
            phx::Math::TVec4<float> m_normal ;

        public:
            Contact(const Contact& contact) 
                : m_time(contact.m_time)
				, m_normal(contact.m_normal) 
            {
			}

            Contact() 
                : m_time(0.0f)
            {
				m_normal.Zero() ;
			}
              
            ~Contact()
            {}

            void SetTime(float time) { m_time = time ; } 
            void SetNormal(phx::Math::TVec4<float> normal) 
            { 
                m_normal = normal ; 
                m_normal.Normalize() ;
            }
              
            float GetTime() { return m_time ; }
            const phx::Math::TVec4<float>& GetNormal() const { return m_normal ; }

            virtual void Resolve(float dt) = 0 ;
        } ;

        class ContactPointSegment : public Contact
        {
        protected:
            phx::Particle* m_a ;
            phx::Particle* m_b ;
            phx::Particle* m_p ;

        public:
            ContactPointSegment(phx::Particle* a, phx::Particle* b, phx::Particle* p)
                : phx::Collision::Contact()
                , m_a(a)
                , m_b(b)
                , m_p(p)
            {
            }

            ContactPointSegment(const ContactPointSegment& contact)
                : phx::Collision::Contact(contact)
                , m_a(contact.m_a)
                , m_b(contact.m_b)
                , m_p(contact.m_p)
            {
            }

            virtual void Resolve(float dt) ;
            virtual bool Detect(float radius) ;
        } ;

        class ContactPointPoint : public Contact
        {
        protected:
            phx::Particle* m_a ;
            phx::Particle* m_b ;

        public:
            ContactPointPoint(phx::Particle* a, phx::Particle* b)
                : phx::Collision::Contact()
                , m_a(a)
                , m_b(b)
            {
            }

            ContactPointPoint(const ContactPointPoint& contact)
                : phx::Collision::Contact(contact)
                , m_a(contact.m_a)
                , m_b(contact.m_b)
            {
            }

            virtual void Resolve(float dt) ;
            virtual bool Detect(float radius) ;
        } ;
    } ;
}