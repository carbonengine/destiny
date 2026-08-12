// Copyright © 2014 CCP ehf.

// stdafx.cpp : source file that includes just the standard includes
//	destiny.pch will be the pre-compiled header
//	stdafx.obj will contain the pre-compiled type information

#include "StdAfx.h"

// TODO: reference any additional headers you need in STDAFX.H
// and not in this file

#if !_HAS_EXCEPTIONS
// throw -- terminate on thrown exception REPLACEABLE
#include <exception>
_STD_BEGIN

_CRTIMP2 _Prhand _Raise_handler = 0;	// define raise handler pointer as null

_CRTIMP2 void __cdecl _Throw(const exception& ex)
	{	// report error and die
		CCP_ASSERT(0);
	}
_STD_END

#endif