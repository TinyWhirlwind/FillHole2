#ifndef _GLDrawFont_
#define _GLDrawFont_

#include <Windows.h>

/* GL������ 
*	\note
*		������ʾʹ��
*/
class GLDrawFont
{
public:
	GLDrawFont()
	{
		bBuilded = false;
	}
	~GLDrawFont()
	{
		KillFont();
	}

	void BuildFont(HDC hDC);  // ����λͼ����					
	void KillFont(void);	  // ɾ����ʾ�б�						
	void GLPrint(const char *str, float x, float y);	// �Զ���GL������庯��
	void GLPrint(const char *str, float x, float y, float z);
	bool IsBuild() { return bBuilded; }

protected:
	unsigned int m_FontList;
	bool	bBuilded;

};

#endif
