/*******************************************************************************

                                moDataManager.h

  ****************************************************************************
  *                                                                          *
  *   This source is free software; you can redistribute it and/or modify    *
  *   it under the terms of the GNU General Public License as published by   *
  *   the Free Software Foundation; either version 2 of the License, or      *
  *  (at your option) any later version.                                    *
  *                                                                          *
  *   This code is distributed in the hope that it will be useful, but       *
  *   WITHOUT ANY WARRANTY; without even the implied warranty of             *
  *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU      *
  *   General Public License for more details.                               *
  *                                                                          *
  *   A copy of the GNU General Public License is available on the World     *
  *   Wide Web at <http://www.gnu.org/copyleft/gpl.html>. You can also       *
  *   obtain it by writing to the Free Software Foundation,                  *
  *   Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.         *
  *                                                                          *
  ****************************************************************************

  Copyright(C) 2006 Fabricio Costa

  Authors:
  Fabricio Costa
  Andr�s Colubri

*******************************************************************************/

#ifndef __MO_DATAMANAGER_H__
#define __MO_DATAMANAGER_H__

#include "moTypes.h"
#include "moText.h"
#include "moConfig.h"
#include "moLock.h"
#include "moResourceManager.h"

/*

class LIBMOLDEO_API moMathFunction : public  moAbstract 
{
public:
protected:
};

template class LIBMOLDEO_API moDynamicArray<moMathFunction*>;
typedef moDynamicArray<moMathFunction*> moMathFunctionArray;


class moCurve : public moMathFunction
{
public:
protected:
};

class moParserFunction : moMathFunction
{
public:
protected:
};

class moTempoFunction : moParserFunction
{
public:
protected:
};*/

class LIBMOLDEO_API moDataSessionConfig {
	public:
		moDataSessionConfig();
		moDataSessionConfig( moText p_datapath, moText p_consoleconfig );
		virtual ~moDataSessionConfig();

		moText GetDataPath();
		moText GetConsoleConfigName();

	private:
		moText m_DataPath;
		moText m_ConsoleConfigName;

};


class LIBMOLDEO_API moDataManager : public moResource
{
	public:

		moDataManager();
		virtual ~moDataManager();

		virtual MOboolean Init();
		virtual MOboolean Init( moText p_datapath, moText p_consoleconfig );
		virtual MOboolean Finish();

		moText GetDataPath();
		moText GetConsoleConfigName();

	protected:

		moDataSessionConfig*	m_pDataSession;
		MOint					m_DataSessionIndex;
};

#endif