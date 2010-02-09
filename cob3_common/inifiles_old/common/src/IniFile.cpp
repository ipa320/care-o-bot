/* -----------------------------------------------------------------------------------
 *
 *		Copyright (c) 2005 Neobotix (www.neobotix.de)
 *
 *      This software is allowed to be used and modified only in association with a Neobotix
 *      robot platform. It is allowed to include the software into applications and
 *      to distribute it with a Neobotix robot platform. 
 *      It is not allowed to distribute the software without a Neobotix robot platform. 
 *
 *      This software is provided WITHOUT ANY WARRANTY; without even the
 *		implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *		PURPOSE. See the Neobotix License (Version 1.0) for more details.
 *
 *      You should have received a copy of the Neobotix License
 *      along with this software; if not, write to the 
 *      Gesellschaft fuer Produktionssysteme, Neobotix, Nobelstrasse 12, 70569 Stuttgart, Germany
 *
 * -----------------------------------------------------------------------------------
 */

//#include "stdafx.h"

#include <IniFile.h>

#include "math.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

//-------------------------------------------------------------------
IniFile::IniFile(): m_vectorSize(500), m_CurCharInd(0)
{
	m_bFileOK=false;
	m_CurLine.resize(m_vectorSize);
}
//--------------------------------------------------------------------------------

IniFile::IniFile(std::string fileName): m_vectorSize(500), m_CurCharInd(0)
{
	m_bFileOK=false;
	m_CurLine.resize(m_vectorSize);
	if(fileName != "") 
		SetFileName(fileName);
}
//--------------------------------------------------------------------------------
IniFile::~IniFile()
{
}
//--------------------------------------------------------------------------------
int IniFile::SetFileName(std::string fileName, std::string strIniFileUsedBy, bool bCreate)
{
//	LOG_OUT("Opening INI-File " << fileName.c_str() << endl);
	m_fileName = fileName;
	m_strIniFileUsedBy = strIniFileUsedBy;

	if ((f = fopen(m_fileName.c_str(),"r")) == NULL)
	{	
		if (bCreate == true)
		{
			f = fopen(m_fileName.c_str(),"w");	// create new file
			LOG_OUT("Creating new INI-File " << m_fileName.c_str() << endl);
			fclose(f);
		}
		else
		{
			LOG_OUT("INI-File not found " << m_fileName.c_str() << endl );
			return -1;
		}
	}
	else fclose(f);
	m_bFileOK = true;
	return 0;
}
//--------------------------------------------------------------------------------
int IniFile::WriteKeyString(const char* pSect, const char* pKey, const std::string* pStrToWrite, bool bWarnIfNotfound)
{	
	std::string StrWithDelimeters = '"' + *pStrToWrite + '"'; 
	return WriteKeyValue(pSect, pKey, StrWithDelimeters.c_str(), bWarnIfNotfound);
}
//--------------------------------------------------------------------------------
int IniFile::WriteKeyValue(const char* szSect,const char* szKey,const char* szValue, bool bWarnIfNotfound)
{
	if (!m_bFileOK) return -1;
	
	FILE* ftemp;
	int   lS,lK,i,bEoff;
	int   bFoundSect,bFoundKey;
	char  c;
	long fpos;


/*------------------------verifications*/

	bFoundSect = 1 /* true*/;
	bFoundKey = 0 /* */;
	bEoff = 0;
	lS = strlen(szSect);
	lK = strlen(szKey);
	if ((lS * lK) == 0) return -1;

/*--------------------- file opening */

	f = fopen(m_fileName.c_str(),"r");
	if (f == NULL)
	{
		LOG_OUT("INI-File not found " << m_fileName.c_str() << endl);
		return -1;
	}
	if ((ftemp = tmpfile()) == NULL)	// nicht unter UNIX (anderes include?)
	{
		LOGAPP_ERR("tmpfile() did not work!");
		/*PrintErr(errno);*/
		return -1;
	}


/* ---------------------- search section and key */
	if (FindSection(szSect, bWarnIfNotfound) != 0)
	{	
		/* fprintf(f,"[%s]\n\n",szSect);*/
		bFoundSect = 0;
	}
	fpos = ftell(f);
	if (bFoundSect)
	{
		if (!FindKey(szKey, false))	bFoundKey = 1;
		fpos = ftell(f);			
	}
	if (feof(f)) bEoff = 1;

/* --------------------- updating the file */
	fseek(f,0,SEEK_SET);
	for (i=0;i<fpos;i++)
	{ 
		fscanf(f,"%c",&c);
		fprintf(ftemp,"%c",c);
		// MMB/23.02.2006: The counter i must not be incremented here on Linux machines!
#ifdef WIN32
		if (c=='\n') 
			i++;
#endif
	};

	if (!bFoundSect) {
		fprintf(ftemp,"\n\n[%s]\n",szSect);
	}
	if (bFoundSect && (!bFoundKey) && (bEoff)) 
	{
		fprintf(ftemp,"\n");
	}
		
	
	if (!bFoundKey) 
	{
		fprintf(ftemp,"%s=",szKey);
	}
	
	fprintf(ftemp,"%s",szValue);

	if (bFoundKey) FindNextLine(m_CurLine, m_CurCharInd);
	if (!(bEoff || feof(f))) 
	{	fprintf(ftemp,"\n");
		while (!feof(f))
		{ 
			fscanf(f,"%c",&c);
			if (!feof(f)) 
			{
				fprintf(ftemp,"%c",c);
			}
		}
	}
	/*fflush(ftemp);*/
	fpos = ftell(ftemp);
	fclose(f);

	if ((f = fopen(m_fileName.c_str(),"w")) == NULL)
	{
		if ((f = fopen(m_fileName.c_str(),"r")) != NULL)
		{
			fclose(f);
			LOG_OUT("INI-File is write protected " << m_fileName.c_str() << endl);
			return -1;
		}

		LOG_OUT("INI-File not found " << m_fileName.c_str() << endl);
		return -1;
	}
	fseek(ftemp,0,SEEK_SET);
	for (i=0;i<fpos;i++)
	{ 
		fscanf(ftemp,"%c",&c);
		fprintf(f,"%c",c);
	};
	fclose(f);
	fclose(ftemp);
	// _rmtmp(); unter UNIX automatisch
	
	return 0;

}
//--------------------------------------------------------------------------------
int IniFile::WriteKeyBool(const char* pSect, const char* pKey, bool bValue, bool bWarnIfNotfound)
{	if(bValue)
		return WriteKeyValue(pSect, pKey, "true", bWarnIfNotfound);
	else
		return WriteKeyValue(pSect, pKey, "false", bWarnIfNotfound);
}
//--------------------------------------------------------------------------------
int IniFile::WriteKeyInt(const char* szSect,const char* szKey,int nValue, bool bWarnIfNotfound)
{
	char buff[20];
	snprintf(buff, 10, "%d", nValue);
	return WriteKeyValue(szSect,szKey,buff, bWarnIfNotfound);
}
//--------------------------------------------------------------------------------
int IniFile::WriteKeyDouble(const char* szSect,const char* szKey,double Value,
						   int Length/*=12*/,int decimals/*=5*/, bool bWarnIfNotfound)
{
	char buff[100];
	sprintf(buff, "%g", Value);
	return WriteKeyValue(szSect,szKey, buff, bWarnIfNotfound);
}
//--------------------------------------------------------------------------------
int IniFile::GetKeyBool(const char* pSect, const char* pKey, bool* pValue, 
							bool bWarnIfNotfound)
{
	std::string strRead;
	char pBuf[20];
	*pValue = false;
	if (GetKeyValue(pSect, pKey, pBuf, 20, bWarnIfNotfound) == -1)
		return -1;

	char* pChar = pBuf;
	while( *pChar == ' ' )	pChar++;		// skip spaces

	if( strncmp(pChar, "true", 4) == 0 )		// is the same?
	{
		*pValue = true;
		return 0;
	}
	if( strncmp(pChar, "false", 5) == 0 )		// is the same?
	{
		*pValue = false;
		return 0;
	}

	return -1;
}
//--------------------------------------------------------------------------------
int IniFile::GetKeyInt(const char* szSect,const char* szKey,int* pValue, 
							bool bWarnIfNotfound)
{
	char buf[9];
	if (GetKeyValue(szSect,szKey,buf,9, bWarnIfNotfound)!=-1)
	{
		//first get rid of spaces
		if (buf[0] == ' ')
		{
			for (int i=0; i<=6; i++)
				buf[i] = buf[i+1];
			buf[7]='\0';
		}
	
 		if ((buf[0]=='0') && (buf[1] == 'x'))
                {
			int iNumLength;
			//check how long the hex-number is
			for (int z=0; z<=7; z++)
			{
				//if its not a number..
				if ((buf[z+2]<0x30) || (buf[z+2]>0x3A))
				{ 	
					iNumLength = z-1;
					break;
				}
			}
			//LOG_OUT("**** Converting >>" << buf << "<< into int");
			*pValue = 0;
                        for (int i=0; i<=(iNumLength); i++)
                        {	
				//convert hex-string from character into int
				*pValue += (buf[i+2]-0x30) * (int)pow((double)16, (iNumLength-i));
				//LOG_OUT("**** length = " << iNumLength + 1 << " Result of char "<<(buf[i+2]-0x30)<<"* 16 ^"<<				iNumLength-i << " = " <<((buf[i+2]-0x30) * (pow(16,(iNumLength-i)))) );
                        }
                }
                else
                {
                        *pValue = atoi(buf);
                }
                return 0;

	}
	else return -1;
}
//--------------------------------------------------------------------------------
int IniFile::GetKeyLong(const char* szSect,const char* szKey,long* pValue, 
							bool bWarnIfNotfound)
{
	char buf[9];
	if (GetKeyValue(szSect,szKey,buf,9, bWarnIfNotfound)!=-1)
	{
		*pValue = atol(buf);
		return 0;
	}
	else return -1;
}
//--------------------------------------------------------------------------------
int IniFile::GetKeyDouble(const char* szSect,const char* szKey,double* pValue, 
							bool bWarnIfNotfound)
{
	char buf[50];
	if (GetKeyValue(szSect, szKey, buf, 50, bWarnIfNotfound) == -1)
	{
		if( bWarnIfNotfound )
			LOG_OUT("Setting parameter " << szKey <<" = " << *pValue << " of section '" << szSect << 
											"' in File '" << m_fileName.c_str() << endl);
		return -1;
	}

	*pValue = atof(buf);
	return 0;
} 
//-----------------------------------------------
int IniFile::GetKeyDouble(const char* pSect,const char* pKey,double* pValue,
						  double dDefault, bool bWarnIfNotfound)
{
	(*pValue) = dDefault;
	return GetKeyDouble(pSect, pKey, pValue, bWarnIfNotfound);
}
//--------------------------------------------------------------------------------
int IniFile::GetKeyValue(const char* szSect,const char* szKey,char* szBuf,
						int lenBuf,	bool bWarnIfNotfound)
{
	if (!m_bFileOK) return -1;

	int   lS,lK;

	lS = strlen(szSect);
	lK = strlen(szKey);
	if ((lS * lK) == 0) return -1;
	if ((f = fopen(m_fileName.c_str(),"r")) == NULL)
	{
		LOG_OUT("INI-File not found " << m_fileName.c_str() << endl);
		return -1;
	}
	if ( FindSection(szSect, bWarnIfNotfound) )
	{	fclose(f); 
		return -1;
	}
	if ( FindKey(szKey, bWarnIfNotfound) )
	{	fclose(f); 
		return -1;
	}
	
	if (feof(f))
	{
		fclose(f);
		return -1;
	}

	//----------- read szBuf bytes from file into szKey
	int BytesRead = fread( szBuf, 1, lenBuf, f );

	// terminate string
	int StrLen;
	if(BytesRead < lenBuf)
	{
		if( BytesRead == 0 && (!feof(f)) )
		{
			LOG_OUT("file read" << endl);
		}
		StrLen = BytesRead;
	}
	else
	{	//assert(BytesRead == lenBuf);
		StrLen = lenBuf-1;
	}
	szBuf[StrLen] = '\0';	
	
	fclose(f);
	return StrLen;
}
//--------------------------------------------------------------------------------
int IniFile::GetKeyString(const char* szSect,const char* szKey, std::string* pStrToRead, 
							bool bWarnIfNotfound)
{
	if (!m_bFileOK) return -1;

	int   lS,lK;

	lS = strlen(szSect);
	lK = strlen(szKey);
	if ((lS * lK) == 0) return -1;
	if ((f = fopen(m_fileName.c_str(),"r")) == NULL)
	{
		LOG_OUT("INI-File not found " << m_fileName.c_str() << endl);
		return -1;
	}
	if ( FindSection(szSect, bWarnIfNotfound) )
	{	fclose(f); 
		return -1;
	}
	if ( FindKey(szKey, bWarnIfNotfound) )
	{	fclose(f); 
		return -1;
	}
	
	if (feof(f))
	{
		fclose(f);
		return -1;
	}

	//----------- read szBuf bytes from file into szKey
	int res = SkipLineUntil(f, '"');			// find beginning of string
	if(res == -1)
	{	if(bWarnIfNotfound)
		{
			LOG_OUT("GetKeyString section " << szSect << " key " << szKey << " first \" not found" << endl);
		}
		fclose(f);
		return -1;
	}

	std::string strRead;
	res = ReadLineUntil(f, '"', strRead);	// read string
	if(res == -1)
	{	
		if(bWarnIfNotfound)
		{	
			LOG_OUT("GetKeyString section " << szSect << " key " << szKey << " string not found" << endl);
		}
		fclose(f);
		return -1;
	}

	// success
	*pStrToRead = strRead;
	fclose(f);
	return 0;
}
//--------------------------------------------------------------------------------
int IniFile::SkipLineUntil(FILE* pFile, const char EndChar)
{
	//assert(pFile);
	int CharsRead = 0;			
	while (1)
	{
		int Char = fgetc(pFile);
		
		if (Char == EndChar)			// end found?
			return CharsRead;			// read finished
	
		if (Char == EOF || Char == '\n')
			return -1;				// end not found

		CharsRead++;
	}
}
//--------------------------------------------------------------------------------
int IniFile::ReadLineUntil(FILE* pFile, const char EndChar, std::string& ReadIntoStr)
{
	//assert(pFile);
	int CharsRead = 0;			
	while (1)
	{
		int Char = fgetc(pFile);
		
		if (Char == EndChar)			// end found?
			return CharsRead;			// read finished
	
		if (Char == EOF || Char == '\n')
			return -1;				// end not found

		ReadIntoStr.append(1, char(Char));		// todo: not very efficient?

		CharsRead++;
	}
}
//--------------------------------------------------------------------------------
int IniFile::FindNextLine(std::vector<char>& NewLine, int& CharInd)
{
	if (!feof(f))
	{
		fgets(&NewLine[0], NewLine.size(), f); // store next line in NewLine
		CharInd=0;        // makes CharInd reference the first char of the line
		return 0;
	}
	return -1;
}
//--------------------------------------------------------------------------------
int IniFile::FindNextSection(std::string* pSect, std::string prevSect, bool bWarnIfNotfound)
{
	std::vector<char> line;
	int charInd = 0;
	int res = -1;

	if (!m_bFileOK) return -1;

	// Make sure that there is no old data.
	pSect->erase();

/*--------------------- file opening */
	f = fopen(m_fileName.c_str(),"r");
	if (f == NULL)
	{
		LOG_OUT("INI-File not found " << m_fileName.c_str() << endl);
		return -1;
	}
	if (feof(f)) return -1;

/*--------------------- search the section */
	if( prevSect != "" ) {
		FindSection( prevSect.c_str(), bWarnIfNotfound );
	} else {
		fseek(f,0,SEEK_SET);
	}

	FindNextLine(m_CurLine, m_CurCharInd);           //read first line of file           
	do
	{
		if (m_CurLine[0] == '[')
		{
			while( m_CurCharInd < m_CurLine.size() ) {
				m_CurCharInd++;
				if (m_CurLine[m_CurCharInd] == ']') // if found section name equals searched one
				{
					for( int i=1; i<m_CurCharInd; ++i )
						pSect->append(1, char(m_CurLine[i]));
					return 0;
				}	
			}
		}
		else 
		{
			FindNextLine(m_CurLine, m_CurCharInd);
		}
	}while (!feof(f));/*do */

/*--------------------- file closing */
	fclose(f);

	return 0;
}
//--------------------------------------------------------------------------------
int IniFile::FindSection(const char* sect, 
							bool bWarnIfNotfound)
{
	int   lS;
	lS = strlen(sect);
	if (feof(f)) return -1;
  
	FindNextLine(m_CurLine, m_CurCharInd);           //read first line of file           
	do
	{
		if (m_CurLine[0] == '[')
		{
			m_CurCharInd++;
			if ((strncmp(&m_CurLine[m_CurCharInd], sect, lS) == 0) && (m_CurLine[m_CurCharInd+lS] == ']')) // if found section name equals searched one
	        {                     
				return 0;
			}	
			else 
			{
				FindNextLine(m_CurLine, m_CurCharInd);
			}
		}
		else if (m_CurLine[m_CurCharInd] == ' ') // if a blank is found
		{
			m_CurCharInd++;
		}
		else 
		{
			FindNextLine(m_CurLine, m_CurCharInd);
		}
	}while (!feof(f));/*do */

	// not found
	if(bWarnIfNotfound)
	{
		LOG_OUT("Section [" << sect << "] in IniFile " << m_fileName.c_str() << " used by "
			<< m_strIniFileUsedBy << " not found" << endl);
	}
	
	return -1;
}
//--------------------------------------------------------------------------------
int IniFile::FindKey(const char* skey, 
							bool bWarnIfNotfound)
{
	int   lS;
	long  fpos = 0l;
	lS = strlen(skey);
	if (feof(f)) return -1;
	
	do
	{
		fpos=ftell(f);             // pointer to the begin of the last read line
		FindNextLine(m_CurLine, m_CurCharInd);
		
		while ( m_CurLine[m_CurCharInd] == ' ' ) // skip blanks
			{
				m_CurCharInd++;
				fpos++;
			}
		
		if (m_CurLine[m_CurCharInd] == '[')		// next section?
			break;		// not found

		if (strncmp(&m_CurLine[m_CurCharInd], skey, lS) == 0)//Found
		{   
			m_CurCharInd+=lS;
			fpos+=lS;                 // set file pointer to end of found key
            while ( m_CurLine[m_CurCharInd] == ' ' ) // skip blanks
			{
				m_CurCharInd++;
				fpos++;
			}
			if ( m_CurLine[m_CurCharInd] == '=' )
			{
				m_CurCharInd++;          // set index to first char after the =
				fpos++;         
				fseek(f,fpos,SEEK_SET);// set file pointer to first char after the =
				return 0;
			}
			
		}
		
	}while (!feof(f));/*do */

	if(bWarnIfNotfound)
	{
		LOG_OUT("Key " << skey << " in IniFile '" << m_fileName.c_str() << "' used by "
			<< m_strIniFileUsedBy << " not found" << endl);
	}
	return -1;
}
//-----------------------------------------------
int IniFile::GetKey(const char* pSect,const char* pKey, std::string* pStrToRead, bool bWarnIfNotfound)
{
	return GetKeyString(pSect,pKey,pStrToRead,bWarnIfNotfound);
}
//-----------------------------------------------
int IniFile::GetKey(const char* pSect,const char* pKey,int* pValue, bool bWarnIfNotfound)
{
	return GetKeyInt(pSect,pKey,pValue,bWarnIfNotfound);
}
//-----------------------------------------------

int IniFile::GetKey(const char* pSect, const char* pKey, bool* pValue, bool bWarnIfNotfound)
{
	return GetKeyBool(pSect,pKey,pValue,bWarnIfNotfound);
}
//-----------------------------------------------
int IniFile::GetKey(const char* pSect,const char* pKey,double* pValue, bool bWarnIfNotfound)
{
	return GetKeyDouble(pSect,pKey,pValue,bWarnIfNotfound);
}

