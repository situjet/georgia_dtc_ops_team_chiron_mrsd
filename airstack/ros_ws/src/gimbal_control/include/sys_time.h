#pragma once
#ifdef _WIN32
#include <windows.h>
struct timeval {
	long tv_sec;
	long tv_usec;
};
inline int gettimeofday(struct timeval* tp, void* tzp) {
	FILETIME ft;
	unsigned __int64 tmpres = 0;
	static const unsigned __int64 EPOCH = ((unsigned __int64)116444736000000000ULL);
	GetSystemTimeAsFileTime(&ft);
	tmpres |= ft.dwHighDateTime;
	tmpres <<= 32;
	tmpres |= ft.dwLowDateTime;
	tmpres -= EPOCH;
	tmpres /= 10;
	tp->tv_sec = (long)(tmpres / 1000000UL);
	tp->tv_usec = (long)(tmpres % 1000000UL);
	return 0;
}
#else
#include <sys/time.h>
#endif
