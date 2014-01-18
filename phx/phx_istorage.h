#pragma once

//
//#include <vector>

namespace phx
{
   class IStorage
   {
   public:
      enum eType
      {
         E_NONE,
         E_SRC,
         E_DST
      } ;

   protected:
      eType m_type ;
      int m_version ;

   public:
      IStorage(eType type) 
         : m_type(type)
         , m_version(1) 
      {
      }

      virtual ~IStorage() 
      {}

      virtual void Write(bool value) = 0 ;
      virtual void Write(int value) = 0 ;
      virtual void Write(long value) = 0 ;
      virtual void Write(size_t value) = 0 ;
      virtual void Write(float value) = 0 ;
      virtual void Write(const char* value) = 0 ;

      virtual void Read(bool& value) = 0 ;
      virtual void Read(int& value) = 0 ;
      virtual void Read(long& value) = 0 ;
      virtual void Read(size_t& value) = 0 ;
      virtual void Read(float& value) = 0 ;
      virtual void Read(char*& value) = 0 ;

      bool IsSource() const { return m_type == E_SRC ; }
      int Version() const { return m_version ; } 
   } ;
}