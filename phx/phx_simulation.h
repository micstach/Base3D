#pragma once

#include "phx\phx_tvec4.h"
#include "phx\phx_base.h"

#include <vector>
#include <map>
#include <set>

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
   class Simulation
   {
   public:
       const float CONST_RELATIVE_VELOCITY_THRESHOLD ;
       const float CONST_ELASTIC_FACTOR ;
       const float CONST_FRICTION_FACTOR ;

   public:
      enum eParticle
      {
         E_POINT = 0, 
         E_POINT_EX = 1,
         E_POINT_FIXED = 2
      } ;

   protected:

       unsigned long m_frame; 

       float m_dt ;
       int m_constraintsResolveSteps ;

       std::vector<Base*> m_objects ;
       std::vector<Particle*> m_particles ;
       std::vector<Constraints::Constraint*> m_constraints ;
      
       std::map<unsigned long, bool> m_connected ;

       std::vector<phx::Base*> m_found ;
       std::set<phx::Base*> m_selection ;

   public:
       void SetConnected(int uidA, int uidB, bool state) ;
       bool IsConnected(int uidA, int uidB) ;

   public:
      Simulation() ;
      virtual ~Simulation() ;

      void Create(eParticle type, Particle** particle) ;
      void Create(const char* type, Constraints::Constraint** constraint) ;
      Constraints::Constraint* Create(const char* type) ;

      Particle* GetParticle(size_t idx) 
      { 
          return m_particles[idx] ; 
      }

      Particle* FindParticle(size_t id) ;
      int GetParticlesCount() { return (int) m_particles.size() ; }
      void SetDt(float dt) ;
      void ResolveConstraints() ;
      void Initialize() ; 
      void Reset() ;

      unsigned long GetFrame() { return m_frame; } 

      float GetDtSq() { return m_dt * m_dt ; }
	  float GetDt() { return m_dt ; }
      int GetConstraintsResolveSteps() { return m_constraintsResolveSteps ; }

      void Find(phx::Math::TVec4<float>& point, Constraints::Constraint** constraint) ;
      phx::Base* Find(phx::Math::TVec4<float>& point) ;

      std::vector<phx::Base*>& Find(phx::Math::TVec4<float>& a, phx::Math::TVec4<float>& b) ;

      Particle* FindParticle(phx::Math::TVec4<float>& point) ;
      Particle* FindParticle(const phx::Math::Ray& ray) ;

      void Delete(phx::Base** object) ;
      void Delete(std::set<Base*>& objects) ;

      virtual void Render(IRenderer* renderer) ;
      virtual void Serialize(IStorage* storage) ;

      int GetUniqueID() ;

      class Physics
      {
      public:
          static float Impulse(float dt, float e, float relativeVelocityFactor, float mass1, float mass2) ;
      } ;

   } ;


}