#ifndef RNDFOBJ_H
#define RNDFOBJ_H

#include "rndf.h"
#include "mem_map.h"
#include <string.h>
#include <signal.h>

#define RNDF_NAME "/rndf"
#define RNDFOBJ_MANAGER RNDFOBJ::GetInstance()

class RNDFOBJ
{
private:
	static RNDFOBJ m_instance;
	pthread_spinlock_t lock;
	MemMap mm;
	void * m_addr;

	static inline void Lock(pthread_spinlock_t* lock);
	static inline void Unlock(pthread_spinlock_t* lock);

	RNDFOBJ();
	~RNDFOBJ();


public:
	bool Get_m_addr();
	bool Initialize();
	void Destroy();
	bool Get(RNDF* rndf);
	static RNDFOBJ& GetInstance() {
		return m_instance;
	}

};

#endif // RNDFOBJ_H
