//
// Copyright (c) 2017 CNRS
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_solvers_fwd_hpp__
#define __invdyn_solvers_fwd_hpp__

#include <memory>

#include "tsid/config.hh"
#include "tsid/math/fwd.hpp"
#include <pinocchio/container/aligned-vector.hpp>

#define DEFAULT_HESSIAN_REGULARIZATION 1e-8

namespace tsid
{
  namespace solvers
  {
    
    /**
     * Available HQP solvers.
     */
    enum TSID_DLLAPI SolverHQP
    {
      SOLVER_HQP_EIQUADPROG = 0,
      SOLVER_HQP_EIQUADPROG_FAST = 1,
      SOLVER_HQP_EIQUADPROG_RT = 2
#ifdef TSID_QPMAD_FOUND
      ,SOLVER_HQP_QPMAD
#endif
#ifdef QPOASES_FOUND
      ,SOLVER_HQP_OASES
#endif
    };
    
    
    /**
     * Possible states of an HQP solver.
     */
    enum TSID_DLLAPI HQPStatus
    {
      HQP_STATUS_UNKNOWN=-1,
      HQP_STATUS_OPTIMAL=0,
      HQP_STATUS_INFEASIBLE=1,
      HQP_STATUS_UNBOUNDED=2,
      HQP_STATUS_MAX_ITER_REACHED=3,
      HQP_STATUS_ERROR=4
    };
    
    class HQPOutput;
    class ProxQPOutput;
    
    class TSID_DLLAPI SolverHQPBase;
    
    template<int nVars, int nEqCon, int nIneqCon>
    class TSID_DLLAPI SolverHQuadProgRT;
    
    template<typename T1, typename T2>
    class aligned_pair
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      aligned_pair(const T1 & t1, const T2 & t2) : first(t1), second(t2) {}
      
      T1 first;
      T2 second;
      
      bool operator==(const aligned_pair & other) const
        { return first == other.first && second == other.second; } 

    };
    
    template<typename T1, typename T2>
    inline aligned_pair<T1,T2> make_pair(const T1 & t1, const T2 & t2)
    { return aligned_pair<T1,T2>(t1,t2); }
    
    template <typename scalar_>
    struct QPGenericDataTpl
    {
      typedef Eigen::Matrix<scalar_,Eigen::Dynamic,1> Vector;
      typedef Eigen::Matrix<scalar_,Eigen::Dynamic,Eigen::Dynamic> Matrix;

      Matrix H;  // cost to minimize
      Vector g;
      Matrix CE;  // eq constraints
      Vector ce0;
    };

    template <typename scalar_>
    struct QPDataTpl : QPGenericDataTpl<scalar_>
    {
      typedef Eigen::Matrix<scalar_,Eigen::Dynamic,1> Vector;
      typedef Eigen::Matrix<scalar_,Eigen::Dynamic,Eigen::Dynamic> Matrix;

      Matrix CI;  // ineq constraints
      Vector ci_lb;  // lower bound
      Vector ci_ub;  // upper bound
    };

    template <typename scalar_>
    struct QPEigquadprogDataTpl : QPGenericDataTpl<scalar_>
    {
      typedef Eigen::Matrix<scalar_,Eigen::Dynamic,1> Vector;
      typedef Eigen::Matrix<scalar_,Eigen::Dynamic,Eigen::Dynamic> Matrix;

      Matrix CI;  // ineq constraints, one-sided
      Vector ci0;  // stack of lower abd upper bounds
    };

    typedef pinocchio::container::aligned_vector< aligned_pair<double, std::shared_ptr<math::ConstraintBase> > > ConstraintLevel;
    typedef pinocchio::container::aligned_vector< aligned_pair<double, std::shared_ptr<const math::ConstraintBase> > > ConstConstraintLevel;
    typedef pinocchio::container::aligned_vector<ConstraintLevel> HQPData;
    typedef pinocchio::container::aligned_vector<ConstConstraintLevel> ConstHQPData;
    
    typedef QPDataTpl<double> QPData;
    
  }
}

#endif // ifndef __invdyn_solvers_fwd_hpp__
