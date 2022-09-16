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

#ifndef __invdyn_solvers_hqp_eiquadprog_fast_hpp__
#define __invdyn_solvers_hqp_eiquadprog_fast_hpp__

#include "tsid/solvers/solver-HQP-base.hpp"
#include "eiquadprog/eiquadprog-fast.hpp"

namespace tsid
{
  namespace solvers
  {
    /**
     * @brief
     */
    class TSID_DLLAPI SolverHQuadProgFast : public SolverHQPBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Matrix Matrix;
      typedef math::Vector Vector;
      typedef math::RefVector RefVector;
      typedef math::ConstRefVector ConstRefVector;
      typedef math::ConstRefMatrix ConstRefMatrix;

      SolverHQuadProgFast(const std::string & name);

      void resizeIneq(unsigned int n, unsigned int nin);
      void setIneq(unsigned int & i_in, const std::shared_ptr<math::ConstraintBase> constr);
      void setBounds(unsigned int & i_in, const std::shared_ptr<math::ConstraintBase> constr);

      /** Solve the given Hierarchical Quadratic Program
       */
      const HQPGenericOutput & solve(const HQPData & problemData);

      /** Get the objective value of the last solved problem. */
      double getObjectiveValue();

      /** Set the current maximum number of iterations performed by the solver. */
      bool setMaximumIterations(unsigned int maxIter);

    protected:

      void sendMsg(const std::string & s);

      // <nVars, nEqCon, 2*nIneqCon>
      eiquadprog::solvers::EiquadprogFast m_solver; 

      Matrix m_H;
      Vector m_g;
      Matrix m_CE;
      Vector m_ce0;
      Matrix m_CI;  /// twice the rows because inequality constraints are bilateral
      Vector m_ci0;
      double m_objValue;

      double m_hessian_regularization;

      Eigen::VectorXi m_activeSet;  /// vector containing the indexes of the active inequalities
      int m_activeSetSize;

      QPEigquadprogDataTpl<double> m_qpData;
    };
  }
}

#endif // ifndef __invdyn_solvers_hqp_eiquadprog_fast_hpp__
