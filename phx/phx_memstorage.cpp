#include <stdafx.h>
#include "phx_memstorage.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

void phx::MemStorage::Write(bool value) 
{
   size_t pos = m_memory.size() ;
   m_memory.resize(pos + sizeof(bool)) ;
   memcpy(&m_memory[pos], &value, sizeof(bool)) ;
}

void phx::MemStorage::Write(int value) 
{
   size_t pos = m_memory.size() ;
   m_memory.resize(pos + sizeof(int)) ;
   memcpy(&m_memory[pos], &value, sizeof(int)) ;
}

void phx::MemStorage::Write(long value) 
{
   size_t pos = m_memory.size() ;
   m_memory.resize(pos + sizeof(long)) ;
   memcpy(&m_memory[pos], &value, sizeof(long)) ;
}

void phx::MemStorage::Write(size_t value) 
{
   size_t pos = m_memory.size() ;
   m_memory.resize(pos + sizeof(size_t)) ;
   memcpy(&m_memory[pos], &value, sizeof(size_t)) ;
}

void phx::MemStorage::Write(float value) 
{
   size_t pos = m_memory.size() ;
   m_memory.resize(pos + sizeof(float)) ;
   memcpy(&m_memory[pos], &value, sizeof(float)) ;
}

void phx::MemStorage::Write(const char* value) 
{
   // write string length
   int strLen = (int) strlen(value) ;
   Write(strLen) ;

   // write string data
   size_t pos = m_memory.size() ;
   m_memory.resize(pos + strLen * sizeof(char)) ;
   memcpy(&m_memory[pos], value, strLen * sizeof(char)) ;
}

void phx::MemStorage::Read(bool& value) 
{
   memcpy(&value, &m_memory[m_position], sizeof(bool)) ;
   m_position += sizeof(bool) ;
}

void phx::MemStorage::Read(int& value) 
{
   memcpy(&value, &m_memory[m_position], sizeof(int)) ;
   m_position += sizeof(int) ;
}

void phx::MemStorage::Read(long& value) 
{
   memcpy(&value, &m_memory[m_position], sizeof(long)) ;
   m_position += sizeof(long) ;
}

void phx::MemStorage::Read(size_t& value) 
{
   memcpy(&value, &m_memory[m_position], sizeof(size_t)) ;
   m_position += sizeof(size_t) ;
}

void phx::MemStorage::Read(float& value) 
{
   memcpy(&value, &m_memory[m_position], sizeof(float)) ;
   m_position += sizeof(float) ;
}

void phx::MemStorage::Read(char*& value) 
{
   int strLen = 0 ;
   Read(strLen) ;

   if (strLen > 0) 
   {
      value = new char [strLen + 1] ;
   }

   memcpy(value, &m_memory[m_position], strLen * sizeof(char)) ;
   value[strLen] = '\0'; 

   m_position += (strLen * sizeof(char)) ;
}