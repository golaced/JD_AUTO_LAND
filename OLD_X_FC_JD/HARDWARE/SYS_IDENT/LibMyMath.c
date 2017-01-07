/*********************************************************************************
*                                ɽè�ɿأ�Lynx��
*                                   ���԰�
*
* Version   	: V1.0
* By        	: Lynx@ustc 84693469@qq.com
*
* For       	: Stm32f103VET6
* Mode      	: Thumb2
* Description   : Ŀǰ���о���㷨����ѧ��
*
*				
* Date          : 2013.XX.XX
*******************************************************************************/
#include "LibMyMath.h"

/*
 * Name										: fconv
 * Description						: float�ľ�������ʽ�˷�������������ȷ����r�ĳ��Ȳ���ɳ�ʼ������ֹr���
 * Entry                  : float a[]���������, int m��a�ĳ���, float b[]���������, int n��b�ĳ���, float r[]������������û����г�ʼ��
 * Return                 : void
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/03/2013
 *
 * create.
 * ----------------------
 */
void fconv(float a[], int m, float b[], int n, float r[])
{
	//������ȷ����r�ĳ��Ȳ���ɳ�ʼ������ֹr���
	int i, j, t = m + n - 1;
	for (i = 0; i < t; i++)
		for (j = 0; j < m && j <= i; j++) 
			if (i - j < n) 
				r[i] += a[j] * b[i - j];
}

