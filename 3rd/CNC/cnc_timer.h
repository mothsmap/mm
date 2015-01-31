/*
 *  CNC: Concurrent Number Cruncher
 *  Copyright (C) 2008 GOCAD/ASGA, INRIA/ALICE
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  If you modify this software, you should include a notice giving the
 *  name of the person performing the modification, the date of modification,
 *  and the reason for such modification.
 *
 *  Contact: Luc Buatois
 *
 *     buatois@gocad.org
 *
 *     ASGA-INPL Bt. G
 *     Rue du Doyen Marcel Roubault - BP 40
 *     54501 VANDOEUVRE LES NANCY
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs.
 */

#ifndef CNC_TIMER_H
#define CNC_TIMER_H

#include <windows.h>

#define OS_WIN

#ifdef OS_WIN
typedef __int64 i64 ;
#endif
#ifdef OS_LINUX
typedef long long i64;
#endif


//---------------------------------------------------------------------------//


class CNCTimer
{

public:
    CNCTimer() ;
    double GetTime() ;
    double GetElapsedTime(double old_time) ;

private:
    i64 _freq ;
    i64 _clocks ;
};

//---------------------------------------------------------------------------//

CNCTimer::CNCTimer() : _clocks(0)
{
    QueryPerformanceFrequency((LARGE_INTEGER *)&_freq);
}

//---------------------------------------------------------------------------//

double CNCTimer::GetTime()
{
    QueryPerformanceCounter((LARGE_INTEGER *)&_clocks);
    return (double)_clocks / (double)_freq;
}

//---------------------------------------------------------------------------//

double CNCTimer::GetElapsedTime(double old_time)
{
    QueryPerformanceCounter((LARGE_INTEGER *)&_clocks);
    return ((double)_clocks / (double)_freq - old_time) ;
}

//---------------------------------------------------------------------------//


#endif // CNC_TIMER_H


