/***********************************************************************
 * This file includes all code required for implementing HDevelop's
 * display operators for HDevEngine/C++.
 *
 * These operators are not included in the HDevEngine/C++ code and they
 * are required to be implemented separately.
 *
 * This implementation below is a direct copy of an example
 * implementation by MVtech.
 *
 * It depends on a portable thread handling implementation, which is
 * also included here.
 *
 ***********************************************************************/




//***********************************************************************
// HDevEngine/C++ example implementation for portable thread handling
// (C) 2005-2013 MVTec Software GmbH
//***********************************************************************
#ifndef MY_THREAD_IMPL_H
#define MY_THREAD_IMPL_H

#ifdef __APPLE__
#include <HALCONCpp/HalconCpp.h>
#else
#include "HalconCpp.h"
#endif

//
// we need a mutex for synchronizing the access to the 
#if defined(WIN32)

// disable warning C4786: symbol greater than 255 character,
#pragma warning(disable : 4786)

#include <windows.h>
#include <process.h>    /* _beginthread, _endthread */

typedef CRITICAL_SECTION MyMutexType;
typedef HANDLE           MyThreadHandleType;

inline Hlong GetThreadId()
{
	return Hlong(GetCurrentThreadId());
}

#else

#include "pthread.h"

typedef pthread_mutex_t MyMutexType;
typedef pthread_t       MyThreadHandleType;

inline Hlong GetThreadId()
{
	return Hlong(pthread_self());
}

#endif


class MyMutexImpl
{
public:

	MyMutexImpl();
	virtual ~MyMutexImpl();
	void Lock();
	void Unlock();

protected:

	MyMutexType mMutex;
};


class MyThreadImpl
{
	typedef Herror (ThreadFunction)(void*);

public:

	MyThreadImpl() : mThreadId(0), mId(0), mThreadFunction(0), mData(0) {}
	void Start(Hlong id, ThreadFunction* function, void* data);
	void Join();

	// returns the thread instance for the currently active thread
	static MyThreadImpl* GetThreadData();

	MyThreadHandleType   mThreadHandle;
	Hlong                mThreadId;
	Hlong                mId;
	ThreadFunction*      mThreadFunction;
	void*                mData;
};

#endif // #ifndef MY_THREAD_IMPL_H
//***********************************************************************


#if defined(WIN32)

#include <process.h>
#include <windows.h>

MyMutexImpl::MyMutexImpl()
{
	InitializeCriticalSection(&mMutex);
}
MyMutexImpl::~MyMutexImpl()
{
	DeleteCriticalSection(&mMutex);
}
void MyMutexImpl::Lock()
{
	EnterCriticalSection(&mMutex);
}
void MyMutexImpl::Unlock()
{
	LeaveCriticalSection(&mMutex);
}

#else

MyMutexImpl::MyMutexImpl()
{
	pthread_mutex_init(&mMutex,NULL);
}
MyMutexImpl::~MyMutexImpl()
{
	pthread_mutex_destroy(&mMutex);
}
void MyMutexImpl::Lock()
{
	pthread_mutex_lock(&mMutex);
}
void MyMutexImpl::Unlock()
{
	pthread_mutex_unlock(&mMutex);
}

#endif
//***********************************************************************


// map that is used for attaching the thread handles to the appropriate
// threads
static std::map<Hlong,MyThreadImpl*>  sThreadMap;
static MyMutexImpl                    sThreadMutex;

// handles of all opened windows, the last one is the active one
static MyThreadImpl* GetThreadData()
{
	sThreadMutex.Lock();
	MyThreadImpl* thread_data = sThreadMap[GetThreadId()];
	sThreadMutex.Unlock();
	return thread_data;
}


// add a new thread to the list of managed threads
//  + this thread is created within another thread (e.g., the main thread),
//    therefor the id must be read from the thread data instead of using
//    GetThreadId()
static void AddThreadData(MyThreadImpl* thread_data)
{
	// no mutex here: lock the whole thread creation and list adding
	sThreadMap[thread_data->mThreadId] = thread_data;
}


// remove a thread from the list of managed threads
//  + this thread is joined within another thread (e.g., the main thread),
//    therefor the id must be read from the thread data instead of using
//    GetThreadId()
static void ClearThreadData(MyThreadImpl* thread_data)
{
	sThreadMutex.Lock();
	sThreadMap.erase(thread_data->mThreadId);
	sThreadMutex.Unlock();
}


#ifdef WIN32

void MyThreadImpl::Start(Hlong id, ThreadFunction* function, void* data)
{
	mId             = id;
	mThreadFunction = function;
	mData           = data;
	unsigned int thread_id;

	// lock the thread mutex from creation of the thread until adding it to the
	// list of managed threads for avoiding that the newly created thread tries
	// to use the thread data before it was added to the list
	sThreadMutex.Lock();
	mThreadHandle = (HANDLE)_beginthreadex(NULL,0,
			(unsigned(__stdcall*)(void*))mThreadFunction,this,0,&thread_id);
	mThreadId = (Hlong)thread_id;
	AddThreadData(this);
	sThreadMutex.Unlock();
}


void MyThreadImpl::Join()
{
	WaitForSingleObject(mThreadHandle,INFINITE);
	ClearThreadData(this);
}


#else


void MyThreadImpl::Start(Hlong id, ThreadFunction* function, void* data)
{
	mId             = id;
	mThreadFunction = function;
	mData           = data;
	// lock the thread mutex from creation of the thread until adding it to the
	// list of managed threads for avoiding that the newly created thread tries
	// to use the thread data before it was added to the list
	sThreadMutex.Lock();
	pthread_create(&mThreadHandle,NULL,(void*(*)(void*))mThreadFunction,this);
	mThreadId       = (Hlong)mThreadHandle;
	AddThreadData(this);
	sThreadMutex.Unlock();
}


void MyThreadImpl::Join()
{
	void* thread_return;
	pthread_join(mThreadHandle,&thread_return);
	ClearThreadData(this);
}

#endif


// handles of all opened windows, the last one is the active one
MyThreadImpl* MyThreadImpl::GetThreadData()
{
	return ::GetThreadData();
}
//***********************************************************************













//***********************************************************************
// HDevEngine/C++ example implementations for HDevelop's display operators.
//
//   This file contains a simple implementation for single-threaded
//   applications and a more complex thread save implementation
//
// (C) 2007-2013 MVTec Software GmbH
//

#ifndef MY_HDEV_OPERATOR_IMPL_H
#define MY_HDEV_OPERATOR_IMPL_H

#ifdef _WIN32
// disable warning C4786: symbol greater than 255 character,
#pragma warning(disable : 4786)
#endif

#ifdef __APPLE__
#include <HALCONCpp/HalconCpp.h>
#include <HDevEngineCpp/HDevEngineCpp.h>
#else
#include "HalconCpp.h"
#include "HDevEngineCpp.h"
#endif

#include <list>
#include <map>




/*****************************************************************************
 * MyHDevOperatorImpl
 *****************************************************************************
 * This is a simple implementation of the HDevelop's display operators.
 ******************************************************************************/
class WindowHandlingImplementation : public HDevEngineCpp::HDevOperatorImplCpp
{
public:

	// Set a default window for programs or procedure that rely on an open
	// window
	void  SetDefaultWindow(Hlong win_id);
	Hlong GetDefaultWindow();

	Hlong       GetCurrentWindow()  const;
	size_t      GetCount()          const;
	void        AddWindow(Hlong id);
	Hlong       PopWindow();
	Hlong       SetWindow(Hlong id);

	// overloaded display operators
	virtual int DevOpenWindow(const HalconCpp::HTuple& row,
			const HalconCpp::HTuple& col,
			const HalconCpp::HTuple& width,
			const HalconCpp::HTuple& height,
			const HalconCpp::HTuple& background,
			HalconCpp::HTuple* win_id);
	virtual int DevSetWindowExtents(const HalconCpp::HTuple& row,
			const HalconCpp::HTuple& col,
			const HalconCpp::HTuple& width,
			const HalconCpp::HTuple& height);
	virtual int DevSetPart(const HalconCpp::HTuple& row1,
			const HalconCpp::HTuple& col1,
			const HalconCpp::HTuple& row2,
			const HalconCpp::HTuple& col2);
	virtual int DevSetWindow(const HalconCpp::HTuple& win_id);
	virtual int DevGetWindow(HalconCpp::HTuple* win_id);
	virtual int DevClearWindow();
	virtual int DevCloseWindow();
	virtual int DevDisplay(const HalconCpp::HObject& obj);
	virtual int DevSetDraw(const HalconCpp::HTuple& draw);
	virtual int DevSetShape(const HalconCpp::HTuple& shape);
	virtual int DevSetColor(const HalconCpp::HTuple& color);
	virtual int DevSetColored(const HalconCpp::HTuple& colored);
	virtual int DevSetLut(const HalconCpp::HTuple& lut);
	virtual int DevSetPaint(const HalconCpp::HTuple& paint);
	virtual int DevSetLineWidth(const HalconCpp::HTuple& width);

protected:

	/***************************************************************************
	 * MyWinIdContainer
	 *   Class that collects all open windows and the currently active window
	 *   of one thread
	 ***************************************************************************/
	class WinIdContainer
	{
	public:
		typedef std::list<Hlong> ListType;

		WinIdContainer();
		void   SetDefaultWindow(Hlong win_id)
		{
			mDefaultId = win_id;
		}
		Hlong  GetDefaultWindow() const
		{
			return mDefaultId;
		}
		Hlong  GetCurrentWindow() const;
		size_t GetCount() const
		{
			return mIds.size();
		}
		void   AddWindow(Hlong id);
		Hlong  PopWindow();
		Hlong  SetWindow(Hlong id);
		ListType::iterator Find(Hlong id);

	protected:
		ListType  mIds;
		Hlong     mDefaultId;
	};

	// handles of all opened windows, the last one is the active one
	WinIdContainer&       GetWinIds();
	const WinIdContainer& GetWinIds() const;

	// map that is used for attaching the window handles to the appropriate
	// threads
	std::map<Hlong,WinIdContainer>  mWinIdsMap;
	MyMutexImpl                     mMutex;
};


#endif // #ifndef MY_HDEV_OPERATOR_IMPL_H
//***********************************************************************


















/*****************************************************************************
 * MyHDevOperatorImpl
 *****************************************************************************
 * This is a simple implementation of the HDevelop's display operators.
 * Attention: This class must not be used in a multi-threaded application
 ******************************************************************************/



#define HCkDev(DEV_OP)        \
		try                           \
		{                             \
			DEV_OP;                     \
			return H_MSG_TRUE;          \
		}                             \
		catch (HException& exc)       \
		{                             \
			return exc.ErrorCode();   \
		}


using namespace HalconCpp;




void WindowHandlingImplementation::SetDefaultWindow(Hlong win_id)
{
	GetWinIds().SetDefaultWindow(win_id);
}


Hlong WindowHandlingImplementation::GetDefaultWindow()
{
	return GetWinIds().GetDefaultWindow();
}


size_t WindowHandlingImplementation::GetCount() const
{
	return GetWinIds().GetCount();
}


void WindowHandlingImplementation::AddWindow(Hlong id)
{
	GetWinIds().AddWindow(id);
}


Hlong WindowHandlingImplementation::PopWindow()
{
	return GetWinIds().PopWindow();
}


Hlong WindowHandlingImplementation::SetWindow(Hlong id)
{
	return GetWinIds().SetWindow(id);
}


// overloaded display operators
int WindowHandlingImplementation::DevOpenWindow(const HTuple& row,
		const HTuple& col,
		const HTuple& width,
		const HTuple& height,
		const HTuple& background,
		HTuple* win_id)
{
	try
	{
		OpenWindow(row,col,width,height,0,"visible","",win_id);
		GetWinIds().AddWindow((*win_id)[0]);
		return H_MSG_TRUE;
	}
	catch (HException& exc)
	{
		return exc.ErrorCode();
	}
}


int WindowHandlingImplementation::DevSetWindowExtents(const HTuple& row,
		const HTuple& col,
		const HTuple& width,
		const HTuple& height)
{
	HCkDev(SetWindowExtents(GetCurrentWindow(),row,col,width,height));
}


int WindowHandlingImplementation::DevSetPart(const HTuple& row1,
		const HTuple& col1,
		const HTuple& row2,
		const HTuple& col2)
{
	HCkDev(SetPart(GetCurrentWindow(),row1,col1,row2,col2));
}


int WindowHandlingImplementation::DevSetWindow(const HTuple& win_id)
{
	GetWinIds().SetWindow(win_id[0].L());
	return H_MSG_TRUE;
}


int WindowHandlingImplementation::DevGetWindow(HTuple* win_id)
{
	if (win_id)
		*win_id = HTuple(GetCurrentWindow());
	return H_MSG_TRUE;
}


int WindowHandlingImplementation::DevClearWindow()
{
	HCkDev(ClearWindow(GetCurrentWindow()));
}


int WindowHandlingImplementation::DevCloseWindow()
{
	// no return, no exception: do not throw an exception if no window is open
			try
	{
				Hlong win_id = GetWinIds().PopWindow();
				if (win_id >= 0)
					CloseWindow(win_id);
	}
			catch (HException&) {}
			return H_MSG_TRUE;
}


int WindowHandlingImplementation::DevDisplay(const HObject& obj)
{
	HCkDev(DispObj(obj,GetCurrentWindow()));
}


int WindowHandlingImplementation::DevSetDraw(const HTuple& draw)
{
	HCkDev(SetDraw(GetCurrentWindow(),draw));
}


int WindowHandlingImplementation::DevSetShape(const HTuple& shape)
{
	HCkDev(SetShape(GetCurrentWindow(),shape));
}


int WindowHandlingImplementation::DevSetColor(const HTuple& color)
{
	HCkDev(SetColor(GetCurrentWindow(),color));
}


int WindowHandlingImplementation::DevSetColored(const HTuple& colored)
{
	HCkDev(SetColored(GetCurrentWindow(),colored));
}


int WindowHandlingImplementation::DevSetLut(const HTuple& lut)
{
	HCkDev(SetLut(GetCurrentWindow(),lut));
}


int WindowHandlingImplementation::DevSetPaint(const HTuple& paint)
{
	HCkDev(SetPaint(GetCurrentWindow(),paint));
}


int WindowHandlingImplementation::DevSetLineWidth(const HTuple& width)
{
	HCkDev(SetLineWidth(GetCurrentWindow(),width));
}


Hlong WindowHandlingImplementation::GetCurrentWindow() const
{
	return GetWinIds().GetCurrentWindow();
}


// handles of all opened windows, the last one is the active one
WindowHandlingImplementation::WinIdContainer& WindowHandlingImplementation::GetWinIds()
{
	mMutex.Lock();
	WinIdContainer& win_ids = mWinIdsMap[GetThreadId()];
	mMutex.Unlock();
	return win_ids;
}


const WindowHandlingImplementation::WinIdContainer& WindowHandlingImplementation::GetWinIds() const
{
	return const_cast<WindowHandlingImplementation*>(this)->GetWinIds();
}




/*****************************************************************************
 * MyWinIdContainer
 *****************************************************************************
 * Class that collects all open windows and the currently active window of one
 * thread
 ******************************************************************************/

WindowHandlingImplementation::WinIdContainer::WinIdContainer()
: mDefaultId(-1)
{
}


Hlong WindowHandlingImplementation::WinIdContainer::GetCurrentWindow() const
{
	return mIds.empty() ? mDefaultId : mIds.back();
}


void WindowHandlingImplementation::WinIdContainer::AddWindow(Hlong id)
{
	mIds.push_back(id);
}


Hlong WindowHandlingImplementation::WinIdContainer::PopWindow()
{
	if (mIds.empty())
		return -1;
	Hlong id = mIds.back();
	mIds.pop_back();
	return id;
}


Hlong WindowHandlingImplementation::WinIdContainer::SetWindow(Hlong id)
{
	ListType::iterator it = Find(id);
	// handle not found
	if (it == mIds.end())
		return H_MSG_FALSE;
	// put handle at end of list
	mIds.erase(it);
	mIds.push_back(id);
	return H_MSG_TRUE;
}


WindowHandlingImplementation::WinIdContainer::ListType::iterator
WindowHandlingImplementation::WinIdContainer::Find(Hlong id)
{
	for (ListType::iterator it = mIds.begin(); it != mIds.end(); ++it)
		if (*it == id)
			return it;
	return mIds.end();
}
//******************************************************************************

