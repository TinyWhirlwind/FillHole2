#ifndef _SN3D_INTERACTOR_
#define _SN3D_INTERACTOR_


/*      sn3DInteractor��

���ඨ���˳���������Ľӿ�,��Ҫ����: 

1.�����Ϣ

2.������Ϣ              */


#include <windows.h>
#include"sn3DSceneGL.h"
//#include"../../sn3DInterface/Isn3DCommandInteractor.h"


namespace sn3DCore
{
	class sn3DInteractor //: public Isn3DCommandInteractor
	{
	 public:
		
	  virtual	void SetSelectRadius(int selectRadius)		=0;  //  ����ѡ��ˢ�뾶
		
	};
}
#endif