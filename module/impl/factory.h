#pragma once

#include "shm.h"

using namespace module::shm;

namespace module
{

namespace factory
{

class SharedObjectsFactory : public SharedObjectsImpl
{
private:
	/*
	 * class-static-variable lazy initialize
	 */
	static SharedObjectsFactory m_instance;

	SharedObjectsFactory()
	{
		m_addr = NULL;
		Initialize();
	}

	virtual ~SharedObjectsFactory()
	{
		Destroy();
	}

	bool Initialize();
	void Destroy();

public:
	static SharedObjectsFactory& GetInstance()
	{
		return m_instance;
	}
};
}
}


