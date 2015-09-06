#ifndef MEMMAP_H
#define MEMMAP_H
#include <stdio.h>  

#include "shm.h"
#include "module.h"

using namespace module;
using namespace module::shm;

#include <unistd.h>  
#include <fcntl.h>  
#include <sys/mman.h>  

class MemMap
{
public:
	MemMap();
	~MemMap();
	bool Map(const char* szFileName);
	void UnMap();
	void SetModule(){
		MarkerData_t markerData;
		markerData.type = MarkerData_t::MARKER_RNDF;
		markerData.value.v_rndf.m_pData = m_pData;
		markerData.value.v_rndf.m_uSize = m_uSize;
		markerData.value.v_rndf.m_nFile = m_nFile;
		SHARED_OBJECTS.SetMarker(markerData);
	}
	void GetModule(){
		MarkerData_t markerData;
		markerData.type = MarkerData_t::MARKER_RNDF;
		SHARED_OBJECTS.GetMarker(&markerData);
		m_pData = markerData.value.v_rndf.m_pData;
		m_uSize = markerData.value.v_rndf.m_uSize;
		m_nFile = markerData.value.v_rndf.m_nFile;
		m_pData = mmap(0, m_uSize, PROT_READ, MAP_SHARED, m_nFile, 0);  
		
	}
	const void* GetData() const {
		return m_pData;
	}
	size_t GetSize() const {
		return m_uSize;
	}

private:
	void*     m_pData;
	size_t    m_uSize;
	int       m_nFile;
};

#endif // MEMMAP_H
