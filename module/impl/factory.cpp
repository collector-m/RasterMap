#include "factory.h"
#include "../log.h"

using namespace module::logger;

namespace module
{
namespace factory
{

SharedObjectsFactory SharedObjectsFactory::m_instance;

bool SharedObjectsFactory::Initialize()
{
	/*
	 * initialize using POSIX shm
	 */
	int shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
	if(shm_fd < 0)
	{
		LOG_WARN_WITH_NO("shm_open");
		return false;
	}

	void* addr = mmap(NULL, sizeof(struct SharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
	if(addr == MAP_FAILED)
	{
		LOG_CRITICAL_WITH_NO("mmap");
		return false;
	}

	this->m_addr = (struct SharedMemory*)addr;

	return true;
}

void SharedObjectsFactory::Destroy()
{
	if(m_addr != NULL)
	{
		munmap(m_addr, sizeof(struct SharedMemory));
		m_addr = NULL;
	}
}

}
}
