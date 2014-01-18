#pragma once

#include "phx\phx_istorage.h"

#include <vector>

namespace phx 
{
   class MemStorage : public IStorage
   {
   protected:
      std::vector<char> m_memory ;
      unsigned long m_position ;

   public:
      MemStorage() 
         : IStorage(IStorage::E_DST)
         , m_position(0)
      {
         _ASSERTE(!IsSource()) ;

          Write(m_version) ;
      }

      MemStorage(void* srcBuffer, size_t length) 
         : IStorage(IStorage::E_SRC)
         , m_position(0)
      {
         _ASSERTE(IsSource()) ;

         m_memory.resize(length) ;
         memcpy((void*) &m_memory[0], srcBuffer, length) ;

         Read(m_version) ;
      }

      virtual ~MemStorage()
      {}

      virtual void Write(bool value) ;
      virtual void Write(int value) ;
      virtual void Write(long value) ;
      virtual void Write(size_t value) ;
      virtual void Write(float value) ;
      virtual void Write(const char* value) ;

      virtual void Read(bool& value) ;
      virtual void Read(int& value) ;
      virtual void Read(long& value) ;
      virtual void Read(size_t& value) ;
      virtual void Read(float& value) ;
      virtual void Read(char*& value) ;

      void* GetBuffer() { return (void*) &m_memory[0] ; }
      unsigned int GetBufferSize() const { return m_memory.size() ; }
      void SetLength(int length) {m_memory.resize(length) ;}
   } ;
} 

