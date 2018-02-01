/*
 *  平成14-19年（独）産業技術総合研究所 著作権所有
 *  
 *  創作者：植芝俊夫
 *
 *  本プログラムは（独）産業技術総合研究所の職員である植芝俊夫が創作し，
 *  （独）産業技術総合研究所が著作権を所有する秘密情報です．創作者によ
 *  る許可なしに本プログラムを使用，複製，改変，第三者へ開示する等の著
 *  作権を侵害する行為を禁止します．
 *  
 *  このプログラムによって生じるいかなる損害に対しても，著作権所有者お
 *  よび創作者は責任を負いません。
 *
 *  Copyright 2002-2007.
 *  National Institute of Advanced Industrial Science and Technology (AIST)
 *
 *  Creator: Toshio UESHIBA
 *
 *  [AIST Confidential and all rights reserved.]
 *  This program is confidential. Any using, copying, changing or
 *  giving any information concerning with this program to others
 *  without permission by the creator are strictly prohibited.
 *
 *  [No Warranty.]
 *  The copyright holders or the creator are not responsible for any
 *  damages in the use of this program.
 *  
 *  $Id: RigidTransformation.cc,v 1.3 2012-09-01 07:23:38 ueshiba Exp $
 */
#include "RigidTransformation.h"

namespace TU
{
/************************************************************************
*  class RigidTransformation						*
************************************************************************/
void
RigidTransformation::update(const vector_type& dD)
{
    for (int i = 0; i < size(); ++i)
	_D[i][size()] -= dD[i];
    switch (size())
    {
      case 2:
      {
	Matrix22d	R(2, 2);
	R[0][0] = R[1][1] = cos(dD[2]);
	R[0][1] = sin(dD[2]);
	R[1][0] = -R[0][1];
	_D(0, 0, 2, 2) = R * _D(0, 0, 2, 2);
      }
	break;
      case 3:
	_D(0, 0, 3, 3) = matrix_type::Rt(dD(3, 3)).trns() * _D(0, 0, 3, 3);
	break;
      default:
	throw std::invalid_argument("RigidTransformation::update(): not implemented for this dimension!!");
	break;
    }
}

}
