#pragma once

namespace phx {
    class Simulation ;
    class IStorage ;
    class IRenderer ;
} ;

#define PHX_CLASS_NAME(name)\
   static const char* ClassName() { return #name ; } \
   virtual const char* GetClassName() { return #name ; } \

namespace phx
{
   class Base
   {
   protected:
      bool m_focus ;
      int m_id ;
      Simulation* m_simulation ;

   public:
      Base(Simulation* simulation) ;
      virtual ~Base() ;   

      virtual void Render(phx::IRenderer* render) = 0 ;
      virtual void Serialize(phx::IStorage* storage) ;
      virtual bool IsReleated(const phx::Base* object) { return false ; }

      // attributes
      virtual void SetFocus(bool focus) ;
      virtual bool IsFocus() const ;
      virtual int GetID() const { return m_id ; }
      virtual Simulation* GetSimulation() { return m_simulation ; }
   } ;
}