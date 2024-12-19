#ifndef _SparseMat_
#define _SparseMat_

#include <vector>
#include <map>

/* SparseMatrix��ʵ����ϡ�����,����˳��ѹ���洢,��Ҫ��������Ԫ��,������ */
class SparseMat
{
 public:

	 class MatRow
	 {
	 public:
		 std::map<int, float> m_Row; ///< Row std::map
	 };

	 SparseMat();

    void Initialize(int rowSize,int colSize,int nnz=0);//�������洢�ռ�
    void Replacement(SparseMat & result);//�����û�
    void Clear();

	//! Gets element.
	/*!
	\param  row Row
	\param  col Column
	\return	 Matrix entry
	*/
	float GetAt(int row, int col);

	//! Sets element.
	/*!
	\param  row Row
	\param  col Column
	\param  entry Entry
	\return	 No return value (void)
	*/
	void SetAt(int row, int col, float entry);
	
	//! Adds element.
	/*!
	\param  row Row
	\param  col Column
	\param  entry Entry
	\return	 No return value (void)
	*/
	void AddAt(int row, int col, float entry);

	//! Get row values.
	/*!
	\param  row Row
	\param  vals Receives values
	\param  idx Receives indices
	\return	 No return value (void)
	*/
	void GetRowVals(int row, std::vector<float>& vals, std::vector<int>& idx);


    int m_rowNum;//����
    int m_colNum;//����
    int m_nnz;//����Ԫ�ؼ���

	std::vector<MatRow> m_Data; ///< array of elements

    std::vector<float>  a;	//������洢�ķ���Ԫ��
    std::vector<int> asub;  //����Ԫ������Ӧ��������
    std::vector<int> xa;    //ÿ�е�һ��Ԫ���� a�е�����

};

//ϡ�����˷�
void Multiply(SparseMat & A, SparseMat & B, SparseMat & result);

#endif