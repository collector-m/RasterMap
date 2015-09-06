#include "rndfobj.h"

RNDFOBJ RNDFOBJ::m_instance;

RNDFOBJ::RNDFOBJ()
{
	m_addr = NULL;
	pthread_spin_init(&lock,PTHREAD_PROCESS_SHARED);
	//Get_m_addr();
}

RNDFOBJ::~RNDFOBJ()
{
}
bool RNDFOBJ::Get_m_addr()
{
	int shm_fd = shm_open(RNDF_NAME, O_RDWR, 0666);
	if(shm_fd < 0) {
		return false;
	}
	void* addr = mmap(NULL, sizeof(int) + sizeof(RNDF), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
	if(addr == MAP_FAILED) {
		return false;
	}
	this->m_addr = addr;

	return true;
}

bool RNDFOBJ::Initialize()
{
	int m_nFile = open("/home/denggroup/UGV/bin/RNDF.txt", O_RDONLY);  
    if (m_nFile < 0)   
    {   
        m_nFile = 0;  
        return false;   
    }  
  
    struct stat status;  
    fstat(m_nFile, &status);  
  
    size_t size = status.st_size;  
	char *m_pData = (char *)mmap(0, size, PROT_READ, MAP_SHARED, m_nFile, 0);  
    if (MAP_FAILED == m_pData) { return false;}  

	return true;
	
/*
	int shm_fd =  shm_open(RNDF_NAME, O_CREAT | O_RDWR | O_TRUNC, 0666);
	if(shm_fd < 0) {
		return false;
	}
	ifstream ifs("/home/wangzhy/UGV/bin/RNDF.txt");
	filebuf* pbuf = ifs.rdbuf();
	size_t size = pbuf->pubseekoff (0,ifs.end,ifs.in);
	pbuf->pubseekpos (0,ifs.in);
	char* buffer=new char[size];
	pbuf->sgetn (buffer,size);
	ifs.close();

	if(ftruncate(shm_fd, sizeof(int)+size) == -1) {
		return false;
	}

	void* addr = mmap(NULL, sizeof(int)+size , PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
	if(addr == MAP_FAILED) {
		return false;
	}

	this->m_addr = addr;

	memset(m_addr, 0, sizeof(int)+size);
	memcpy(m_addr, &size , sizeof(int));
	memcpy(m_addr+sizeof(int), buffer , size);

	return true;
	 * */
}

void RNDFOBJ::Destroy()
{
	if(m_addr != NULL) {
		munmap(m_addr, sizeof(RNDF));
		m_addr = NULL;
	}
}


bool RNDFOBJ::Get(RNDF *rndf)
{
	//struct timeval tv1;
	//struct timeval tv2;
	//struct timezone tz;

	Lock(&lock);
	//gettimeofday(&tv1,&tz);
	int m_nFile = open("RNDF.txt", O_RDONLY);  
    if (m_nFile < 0)   
    {   
        m_nFile = 0;  
        return false;   
    }  
	//gettimeofday(&tv2,&tz);
	//cout << "open:" << 1000*((tv2.tv_sec-tv1.tv_sec)+(tv2.tv_usec-tv1.tv_usec)/1000000.) << endl;
    
	//gettimeofday(&tv1,&tz);
	struct stat status;  
    fstat(m_nFile, &status);  
    size_t size = status.st_size; 
	//gettimeofday(&tv2,&tz);
	//cout << "fstat:" << 1000*((tv2.tv_sec-tv1.tv_sec)+(tv2.tv_usec-tv1.tv_usec)/1000000.) << endl;
	
	//gettimeofday(&tv1,&tz);
    char *m_pData = (char*)mmap(0, size, PROT_READ, MAP_SHARED, m_nFile, 0);  
    if (MAP_FAILED == m_pData) { return false;} 
	//gettimeofday(&tv2,&tz);
	//cout << "mmap:" << 1000*((tv2.tv_sec-tv1.tv_sec)+(tv2.tv_usec-tv1.tv_usec)/1000000.) << endl;
	
	//gettimeofday(&tv1,&tz);
	string str(m_pData,size);
	//gettimeofday(&tv2,&tz);
	//cout << "str:" << 1000*((tv2.tv_sec-tv1.tv_sec)+(tv2.tv_usec-tv1.tv_usec)/1000000.) << endl;
	
	//gettimeofday(&tv1,&tz);
	stringstream ss(str);
	//gettimeofday(&tv2,&tz);
	//cout << "stringstream:" << 1000*((tv2.tv_sec-tv1.tv_sec)+(tv2.tv_usec-tv1.tv_usec)/1000000.) << endl;
	
	//gettimeofday(&tv1,&tz);
	rndf->Read(ss);
	//gettimeofday(&tv2,&tz);
	//cout << "Read:" << 1000*((tv2.tv_sec-tv1.tv_sec)+(tv2.tv_usec-tv1.tv_usec)/1000000.) << endl;
	//cout << str << endl;
	Unlock(&lock);
	/*
	size_t size;
	memcpy(&size, m_addr, sizeof(int));
	char *pbuf = new char[size+1];
	memcpy(pbuf, m_addr+sizeof(int), size);
	pbuf[size] = '\0';
	cout << pbuf << endl;
	string str(pbuf,size);
	stringstream ss(str);
	rndf->Read(ss);
	 * */
}

inline void RNDFOBJ::Lock(pthread_spinlock_t* lock)
{
	sigset_t newset;
	sigemptyset(&newset);
	sigaddset(&newset, SIGINT);
	sigprocmask(SIG_BLOCK, &newset, NULL);

	pthread_spin_lock(lock);
}

inline void RNDFOBJ::Unlock(pthread_spinlock_t* lock)
{
	pthread_spin_unlock(lock);

	sigset_t newset;
	sigemptyset(&newset);
	sigaddset(&newset, SIGINT);
	sigprocmask(SIG_UNBLOCK, &newset, NULL);
}
