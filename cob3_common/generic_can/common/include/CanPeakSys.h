//-----------------------------------------------
// Neobotix 
// www.neobotix.de
// Copyright (c) 2003. All rights reserved.

// author: Oliver Barth
//-----------------------------------------------
#ifndef CANPEAKSYS_INCLUDEDEF_H
#define CANPEAKSYS_INCLUDEDEF_H
//-----------------------------------------------
#include <CanItf.h>
#include <libpcan/libpcan.h>
#include <inifiles_old/IniFile.h>

//-----------------------------------------------

class CanPeakSys : public CanItf
{
public:
	// --------------- Interface
	CanPeakSys(const char* cIniFile);
	~CanPeakSys();
	void init();
	void destroy() {}
	bool transmitMsg(CanMsg CMsg, bool bBlocking = true);
	bool receiveMsg(CanMsg* pCMsg);
	bool receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry);
	bool isObjectMode() { return false; }

private:
	// --------------- Types
	HANDLE m_handle;
	
	bool m_bInitialized;
	IniFile m_IniFile;
	bool m_bSimuEnabled;

	static const int c_iInterrupt;
	static const int c_iPort;
};
//-----------------------------------------------
#endif

