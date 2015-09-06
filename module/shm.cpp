#include "shm.h"
#include "impl/factory.h"

using namespace module::factory;

namespace module
{
namespace shm
{
SharedObjects& SharedObjects::GetInstance()
{
	return SharedObjectsFactory::GetInstance();
}
}
}
