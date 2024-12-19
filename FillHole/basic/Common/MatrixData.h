// File: MatrixData.h

#ifndef _MATRIX_DATA_
#define _MATRIX_DATA_
//#include <Windows.h>
//#include <vector>
#include <stdlib.h>
namespace sn3DCore
{
	// Ϊ������[][]
	template <class  T>  
	class MatrixData     
	{    
	public:
		// ������
		template <class T2>
		class ArrayData
		{
		public:
			// ���صڶ���[]
			T2& operator[](int j)				{ return _p[j]; }
			ArrayData& operator= ( T2* p )		{ _p = p; return *this; }
		protected:
			T2 *_p;
		};

	public: 
		// ����/���� ����
		MatrixData():m_data(NULL),m_rows(NULL)	{}
		MatrixData(int w, int h, T* data):m_data(NULL),m_rows(NULL)		
		{
			Create( w, h, data );
		}
		virtual ~MatrixData()
		{
			if(m_data) 
				delete[] m_data;
			if(m_rows) 
				delete[] m_rows;
		}

		// ��������
		void Create(int w, int h, T* data)
		{
			m_w = w;
			m_h = h;
			m_size = w*h;
			if( m_data ) 
			{
				delete[] m_data;
				m_data = NULL;
			}
			if( m_rows ) 
			{
				delete[] m_rows;
				m_rows = NULL;
			}

			if( data==NULL )
				m_data = new T[m_size];
			else
				m_data = data;
				

			//m_data.resize( m_size );
			//if( data != NULL )
			//{
			//	for (int i=0;i<m_size;i++)
			//		m_data[i] = data[i];
			//}	

			//m_rows.resize( m_h );
			m_rows = new ArrayData<T>[m_h];
			for (int i=0; i<m_h; i++)
				m_rows[i] = &(m_data[i*m_w]);
		}
		void Fill(T& val)
		{
			for(int i=0; i<m_size; i++)
				m_data[i] = val;
		}
		void Release()
		{
			m_w    = m_h = 0;
			m_size = 0;
			if( m_data ) 
				delete[] m_data;
			if( m_rows ) 
				delete[] m_rows;
			m_data = NULL;
			m_rows = NULL;
			//std::vector<T>().swap( m_data );
			//std::vector<ArrayData<T>>().swap( m_rows ); 
		}
 
		// ���ص�һ��[]�� 
		ArrayData<T>& operator[](int i) { return m_rows[i]; } 

		ArrayData<T>& at(int i)			{ return m_rows[i]; }
		T& at(int i, int j)				{ return m_rows[i][j]; }

		// ����ȫ
		MatrixData<T>& operator= (MatrixData<T>& md)
		{
			if( &md != this )
			{
				T* data = new T[md.m_size];
				memcpy( data, md.m_data, md.m_size*sizeof(T));
				Create( md.m_w, md.m_h, data );
			}
			return *this;
		}
		
	protected:
 
		int m_w;
		int m_h;
		int m_size;

		T				* m_data;
		ArrayData<T>	* m_rows;


		//std::vector<T>				m_data;
		//std::vector<ArrayData<T>>	m_rows;     
	};

}

#endif // _MATRIX_DATA_
