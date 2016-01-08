#pragma once

// Check windows
#if _WIN32 || _WIN64
#	if _WIN64
#		define ENV64BIT
#	else
#		define ENV32BIT
#	endif // _WIN64
#endif //_WIN32 || _WIN64

// Check GCC
#if __GNUC__
#	if __x86_64__ || __ppc64__
#		define ENV64BIT
#	else
#		define ENV32BIT
#	endif //__x86_64__ || __ppc64__
#endif //__GNUC__

#ifdef NDEBUG
#	ifndef _HAS_EXCEPTIONS
#		define _HAS_EXCEPTIONS 0
#	endif // _HAS_EXCEPTIONS
#	ifndef WIN32
#		define WIN32
#	endif // WIN32
#	ifndef WIN64
#		define WIN64
#	endif // WIN64
#	ifndef _CRT_SECURE_NO_DEPRECATE
#		define _CRT_SECURE_NO_DEPRECATE
#	endif // _CRT_SECURE_NO_DEPRECATE
#	ifndef _CRT_NONSTDC_NO_DEPRECATE
#		define _CRT_NONSTDC_NO_DEPRECATE
#	endif // _CRT_NONSTDC_NO_DEPRECATE
#	ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#		define _WINSOCK_DEPRECATED_NO_WARNINGS
#	endif // _WINSOCK_DEPRECATED_NO_WARNINGS
#	ifndef PHYSX_PROFILE_SDK
#		define PHYSX_PROFILE_SDK
#	endif // PHYSX_PROFILE_SDK
#else
#	ifndef _HAS_EXCEPTIONS
#		define _HAS_EXCEPTIONS 0
#	endif // _HAS_EXCEPTIONS
#	ifndef WIN32
#		define WIN32
#	endif // WIN32
#	ifndef WIN64
#		define WIN64
#	endif // WIN64
#	ifndef _CRT_SECURE_NO_DEPRECATE
#		define _CRT_SECURE_NO_DEPRECATE
#	endif // _CRT_SECURE_NO_DEPRECATE
#	ifndef _CRT_NONSTDC_NO_DEPRECATE
#		define _CRT_NONSTDC_NO_DEPRECATE
#	endif // _CRT_NONSTDC_NO_DEPRECATE
#	ifndef _WINSOCK_DEPRECATED_NO_WARNINGS
#		define _WINSOCK_DEPRECATED_NO_WARNINGS
#	endif // _WINSOCK_DEPRECATED_NO_WARNINGS
#	ifndef PHYSX_PROFILE_SDK
#		define PHYSX_PROFILE_SDK
#	endif // PHYSX_PROFILE_SDK
#	ifndef _DEBUG
#		define _DEBUG
#	endif // _DEBUG
#	ifndef PX_DEBUG
#		define PX_DEBUG
#	endif // PX_DEBUG
#	ifndef PX_CHECKED
#		define PX_CHECKED
#	endif // PX_CHECKED
#	ifndef PX_SUPPORT_VISUAL_DEBUGGER
#		define PX_SUPPORT_VISUAL_DEBUGGER
#	endif // PX_SUPPORT_VISUAL_DEBUGGER
#	ifndef PX_NVTX
#		define PX_NVTX
#	endif // PX_NVTX
#	ifndef RENDERER_PVD
#		define RENDERER_PVD
#	endif // RENDERER_PVD
#endif



#include <PxPhysicsAPI.h>