/*
 *  $Id: ProjectiveMappingWithDistortion.h,v 1.1 2007-03-08 05:11:09 ueshiba Exp $
 */
#include "TU/Geometry++.h"

namespace TU
{
/************************************************************************
*  class ProjectiveMappingWithDistortion				*
************************************************************************/
class ProjectiveMappingWithDistortion : public ProjectiveMapping
{
  public:
    template <class S, class B>
    ProjectiveMappingWithDistortion(const Vector<S, B>& u0)
	:ProjectiveMapping(), _k(0.0), _u0(u0)				{}

    template <class Iterator>
    void		initialize(Iterator first, Iterator last,
				   bool refine=true)			;
    template <class S, class B>
    Vector<double>	operator ()(const Vector<S, B>& x)	const	;
    template <class S, class B>
    Vector<double>	mapP(const Vector<S, B>& x)		const	;
    template <class S, class B>
    Matrix<double>	jacobian(const Vector<S, B>& x)	const	;
			operator const Vector<double>()		const	;
    u_int		dof()					const	;
    void		update(const Vector<double>& dt)		;
    
  private:
    double		_k;	// distortion coefficient
    Vector<double>	_u0;	// distortion center
};

template <class Iterator> inline void
ProjectiveMappingWithDistortion::initialize(Iterator first,
					    Iterator last, bool refine)
{
    ProjectiveMapping::initialize(first, last, false);

    _k  = 0.0;
    if (refine)
    {
	Cost<ProjectiveMappingWithDistortion, Iterator>	cost(first, last);
	ConstNormConstraint<ProjectiveMappingWithDistortion>
	    constraint(*this);
	minimizeSquare(cost, constraint, *this);
    }
}

template <class S, class B> inline Vector<double>
ProjectiveMappingWithDistortion::operator ()(const Vector<S, B>& x) const
{
    const Vector<double>&	du = ProjectiveMapping::operator ()(x) - _u0;
    return _u0 + (1.0 + _k * du.square()) * du;
}

template <class S, class B> inline Vector<double>
ProjectiveMappingWithDistortion::mapP(const Vector<S, B>& x) const
{
    Vector<double>	u(outDim()+1);
    u(0, outDim()) = (*this)(x);
    u[outDim()]    = 1.0;
    return u;
}

template <class S, class B> inline Matrix<double>
ProjectiveMappingWithDistortion::jacobian(const Vector<S, B>& x) const
{
    const Vector<double>&	du = ProjectiveMapping::operator ()(x) - _u0;
    const double		r2 = du.square();
    Matrix<double>		J(outDim(), dof());

  // _k‚ÉŠÖ‚·‚é”÷•ª
    for (int i = 0; i < J.nrow(); ++i)
	J[i][0] = r2 * du[i];

  // ŽË‰e•ÏŠ·s—ñ‚ÉŠÖ‚·‚é”÷•ª
    const double	tmp = 1.0 + _k * du.square();
    Matrix<double>	K   = (2.0 * _k * du) % du;
    for (int i = 0; i < K.nrow(); ++i)
	K[i][i] += tmp;
    J(0, 1, outDim(), J.ncol() - 1) = K * ProjectiveMapping::jacobian(x);
    
    return J;
}

inline
ProjectiveMappingWithDistortion::operator const Vector<double>() const
{
    Vector<double>	t(dof());
    t(1, dof() - 1) = Vector<double>(const_cast<Matrix<double>&>(T()));
    return t;
}
    
inline void
ProjectiveMappingWithDistortion::update(const Vector<double>& dm)
{
    _k -= dm[0];
    ProjectiveMapping::update(dm(1, dof() - 1));
}

inline u_int
ProjectiveMappingWithDistortion::dof() const
{
    return 1 + ProjectiveMapping::dof();
}
    
}
