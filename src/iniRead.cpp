#include "iniRead.h"
/******************************************************************************
* 功  能：构造函数
* 参  数：无
* 返回值：无
* 备  注：
******************************************************************************/
CIni::CIni()
{
    memset(m_szKey, 0, sizeof(m_szKey));
    m_fp = NULL;
}

/******************************************************************************
* 功  能：析构函数
* 参  数：无
* 返回值：无
* 备  注：
******************************************************************************/

CIni::~CIni()
{
    m_Map.clear();
}

/******************************************************************************
* 功  能：打开文件函数
* 参  数：无
* 返回值：
* 备  注：
******************************************************************************/
INI_RES CIni::OpenFile(const char *pathName)
{
    string szLine, szMainKey, szLastMainKey, szSubKey;
    char strLine[CONFIGLEN] = {0};
    KEYMAP mLastMap;
    int nIndexPos = -1;
    int nLeftPos = -1;
    int nRightPos = -1;
    m_fp = fopen(pathName, "r");

    if (m_fp == NULL)
    {
        printf("open inifile %s error!\n", pathName);
        return INI_OPENFILE_ERROR;
    }

    m_Map.clear();

    while (fgets(strLine, CONFIGLEN, m_fp))
    {
        szLine.assign(strLine);
        //删除字符串中的非必要字符
        nLeftPos = szLine.find("\n");
        if (string::npos != nLeftPos)
        {
            szLine.erase(nLeftPos, 1);
        }
        nLeftPos = szLine.find("\r");
        if (string::npos != nLeftPos)
        {
            szLine.erase(nLeftPos, 1);
        }
        //判断是否是主键
        nLeftPos = szLine.find("[");
        nRightPos = szLine.find("]");
        if (nLeftPos != string::npos && nRightPos != string::npos)
        {
            szLine.erase(nLeftPos, 1);
            nRightPos--;
            szLine.erase(nRightPos, 1);
            trim(szLine);
            m_Map[szLastMainKey] = mLastMap;
            mLastMap.clear();
            szLastMainKey = szLine;
        }
        else
        {
            //是否是子键
            if (nIndexPos = szLine.find("="), string::npos != nIndexPos)
            {
                string szSubKey, szSubValue;
                szSubKey = szLine.substr(0, nIndexPos);
                szSubValue = szLine.substr(nIndexPos + 1, szLine.length() - nIndexPos - 1);
                trim(szSubKey);
                trim(szSubValue);
                mLastMap[szSubKey] = szSubValue;
            }
            else
            {
                //TODO:不符合ini键值模板的内容 如注释等
            }
        }
    }
    //插入最后一次主键
    m_Map[szLastMainKey] = mLastMap;

    return INI_SUCCESS;
}

/******************************************************************************
* 功  能：关闭文件函数
* 参  数：无
* 返回值：
* 备  注：
******************************************************************************/
INI_RES CIni::CloseFile()
{

    if (m_fp != NULL)
    {
        fclose(m_fp);
        m_fp = NULL;
    }

    return INI_SUCCESS;
}

/******************************************************************************
* 功  能：获取[SECTION]下的所有键值的字符串
* 参  数：
*  char* mAttr  输入参数    主键
*  char* value  输出参数 子键键值
* 返回值：
* 备  注：
******************************************************************************/
INI_RES CIni::GetSection(const char *mAttr, std::vector<std::string>& section)
{

    MAINKEYMAP::iterator mainKeyIte = m_Map.find(mAttr);
    if (mainKeyIte == m_Map.end())
        return INI_NO_ATTR;
    
    KEYMAP::iterator keyIte = mainKeyIte->second.begin();
    for (; keyIte!=mainKeyIte->second.end(); keyIte++)
    {
        section.push_back(keyIte->second);
    }
    return INI_SUCCESS;
}

/******************************************************************************
* 功  能：获取[SECTION]下的某一个键值的字符串
* 参  数：
*  char* mAttr  输入参数    主键
*  char* cAttr  输入参数 子键
*  char* value  输出参数 子键键值
* 返回值：
* 备  注：
******************************************************************************/
INI_RES CIni::GetKey(const char *mAttr, const char *cAttr, char *pValue)
{

    MAINKEYMAP::iterator mainKeyIte = m_Map.find(mAttr);
    if (mainKeyIte == m_Map.end())
        return INI_NO_ATTR;

    KEYMAP::iterator keyIte = mainKeyIte->second.find(cAttr);
    if (keyIte == mainKeyIte->second.end())
        return INI_NO_ATTR;

    strcpy(pValue, keyIte->second.c_str());

    return INI_SUCCESS;
}

/******************************************************************************
* 功  能：获取整形的键值
* 参  数：
*       cAttr                     主键
*      cAttr                     子键
* 返回值：正常则返回对应的数值 未读取成功则返回0(键值本身为0不冲突)
* 备  注：
******************************************************************************/
int CIni::GetInt(const char *mAttr, const char *cAttr)
{
    int nRes = 0;

    memset(m_szKey, 0, sizeof(m_szKey));

    if (INI_SUCCESS == GetKey(mAttr, cAttr, m_szKey))
    {
        nRes = atoi(m_szKey);
    }
    return nRes;
}

/******************************************************************************
* 功  能：获取浮点型的键值
* 参  数：
*       cAttr                     主键
*      cAttr                     子键
* 返回值：正常则返回对应的数值 未读取成功则返回0(键值本身为0不冲突)
* 备  注：
******************************************************************************/
float CIni::GetFloat(const char *mAttr, const char *cAttr)
{
    float nRes = 0;

    memset(m_szKey, 0, sizeof(m_szKey));

    if (INI_SUCCESS == GetKey(mAttr, cAttr, m_szKey))
    {
        printf("key:%s \n", m_szKey);
        nRes = atof(m_szKey);
    }
    return nRes;
}

/******************************************************************************
* 功  能：获取键值的字符串
* 参  数：
*       cAttr                     主键
*      cAttr                     子键
* 返回值：正常则返回读取到的子键字符串 未读取成功则返回"NULL"
* 备  注：
******************************************************************************/
char *CIni::GetStr(const char *mAttr, const char *cAttr)
{
    memset(m_szKey, 0, sizeof(m_szKey));

    if (INI_SUCCESS != GetKey(mAttr, cAttr, m_szKey))
    {
        strcpy(m_szKey, "NULL");
    }

    return m_szKey;
}

/******************************************************************************
* 功  能：去除字符串两端的空格
* 参  数：
* 返回值：
* 备  注：
******************************************************************************/
void CIni::trim(std::string &s)
{
    if (s.empty())
    {
        return ;
    }
    s.erase(0, s.find_first_not_of(" "));
    s.erase(s.find_last_not_of(" ") + 1);
}