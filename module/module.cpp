#include "module.h"
#include <stdio.h>
#include <signal.h>
#include "log.h"

using namespace module::logger;

namespace module
{
ModuleCleanup __handler = NULL;

void __int_handler(int sig)
{
	if(__handler != NULL)
		__handler();
	_exit(0);
}

void RegisterCleanup(ModuleCleanup cleanup)
{
	if(__handler == cleanup)
		return;

	__handler = cleanup;

	// ignore SIGPIPE which may be sent by "sendto" system call
//	signal(SIGPIPE, SIG_IGN);

	// set SIGCHLD handler
	// note: this handler will not be restored to default
	//       after the handler is called
	struct sigaction sa;
	sa.sa_flags = SA_RESTART;
	sa.sa_handler = __int_handler;
	sigemptyset(&sa.sa_mask);
	if (sigaction(SIGINT, &sa, NULL) == -1 || sigaction(SIGABRT, &sa, NULL) == -1)
	{
		LOG_ERROR_WITH_NO("sigaction");
	}
}

void UnRegister()
{
	__handler = NULL;
}
}
