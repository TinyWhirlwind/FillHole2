#ifndef  _EDGE_
#define  _EDGE_

/*******************************
  Edge ��

  �����˸��༸�������еı߶���

********************************/

namespace sn3DCore
{
class Edge
{
public:			
////////////////////////////   ���Է���  ///////////////////////////////////

    int & V( const int j){return v[j];}              // ��ȡ����

	int & F( const int j){return f[j];}              // ��ȡ����

	int  &  Index(){return m_index;}   //  ��ö�������

	int m_id;

public:
//////////////////////////////////  ��־λ ///////////////////////////////////

    bool IsD()                             // ɾ��λ 
	{return (_flags & DELETED) != 0;}
	void SetD()	  { _flags |=DELETED;}
	void ClearD() { _flags &= (~DELETED);}

	bool IsS()                             // ѡ��λ 
	{return (_flags & SELECTED) != 0;}
	void SetS()	{_flags |=SELECTED;}
	void ClearS(){_flags &= (~SELECTED);}

	bool IsV()                             // ѡ��λ 
	{return (_flags & VISITED) != 0;}
	void SetV()	{_flags |=VISITED;}
	void ClearV(){_flags &= (~VISITED);}

	bool IsM() 
	{return (_flags & MODIFIED) != 0;}	   // �޸�λ
	void SetM(){_flags |= MODIFIED;}						
	void ClearM(){_flags &= ~MODIFIED;}		

	bool IsB()                             // �߽�λ
	{return (_flags & (BORDER)) != 0;}    
	void SetB()	{_flags |=(BORDER);}
	void ClearB(){_flags &= (~(BORDER));}

	bool IsUserBit(int userBit)            // �û�λ
	{ return (_flags & userBit) != 0;}
	void SetUserBit(int userBit){_flags |=userBit; }
	void ClearUserBit(int userBit){_flags &= (~userBit);}

	static int &LastBitFlag()
	{
	  static int b =USER0;
	  return b;
	}
	static inline int NewBitFlag()
	{
	  LastBitFlag()=LastBitFlag()<<1;
	  return LastBitFlag();
	}
	static inline bool DeleteBitFlag(int bitval)
	{	
	  if(LastBitFlag()==bitval)
	  {
	   LastBitFlag()= LastBitFlag()>>1;
	   return true;
	  }
	  return false;
	}

	int & Flags (){return _flags;}  // ��ȡ��־λ

	void ClearFlags() {_flags=0;}   // �����־λ


	enum{
	  DELETED     = 0x00000001,		// ɾ��λ
	  NOTREAD     = 0x00000002,		// ��ȡλ
	  NOTWRITE    = 0x00000004,		// д��λ
	  SELECTED    = 0x00000008,		// ѡ��λ
	  VISITED     = 0x00000010,		// ����λ
	  MODIFIED    = 0x00000020,		// �޸�λ
	  BORDER      = 0x00000040,     // �߽�λ
	  USER0       = 0x00000080      // �û�λ
	};

public:

    Edge(){}

	Edge(int v0,int v1){v[0]=v0;v[1]=v1;}
	
private:

	int  v[2];    // �˵�
    int  f[2];    // ����
	int  m_index;				// ����������ţ��������������˳��
	int  _flags;  // ��־λ	
	
	
};
}
#endif