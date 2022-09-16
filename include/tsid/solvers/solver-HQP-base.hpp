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

#ifndef __invdyn_solvers_hqp_base_hpp__
#define __invdyn_solvers_hqp_base_hpp__

#include "tsid/solvers/fwd.hpp"
#include "tsid/solvers/solver-HQP-output.hpp"
#include "tsid/math/constraint-base.hpp"

#include <vector>
#include <utility>

namespace tsid
{
  namespace solvers
  {

    /**
     * @brief Abstract interface for a Quadratic Program (HQP) solver.
     */
    class TSID_DLLAPI SolverHQPBase
    {
    public:

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      static std::string const HQP_status_string [5];

      typedef math::RefVector RefVector;
      typedef math::ConstRefVector ConstRefVector;
      typedef math::ConstRefMatrix ConstRefMatrix;
      typedef math::ConstraintInequality ConstraintInequality;

      SolverHQPBase(const std::string & name);
      virtual ~SolverHQPBase() {};

      virtual const std::string & name() { return m_name; }

      virtual bool resize(unsigned int n, unsigned int neq, unsigned int nin);

      /** function to treat inequalitiy constraints differently, depending if unilateral or not */
      virtual void resizeIneq(unsigned int n, unsigned int nin);
      virtual void setIneq(unsigned int & i_in, const std::shared_ptr<math::ConstraintBase> constr);
      virtual void setBounds(unsigned int & i_in, const std::shared_ptr<math::ConstraintBase> constr);


      virtual void sendMsg(const std::string & s) = 0;


      /** Solve the specified Hierarchical Quadratic Program.
       */
      virtual const HQPGenericOutput & solve(const HQPData & problemData) = 0;

      virtual void retrieveQPData(const HQPData & problemData, 
                                  const bool hessianRegularization = true);

      /** Get the objective value of the last solved problem. */
      virtual double getObjectiveValue() = 0;

      /** Return true if the solver is allowed to warm start, false otherwise. */
      virtual bool getUseWarmStart(){ return m_useWarmStart; }
      /** Specify whether the solver is allowed to use warm-start techniques. */
      virtual void setUseWarmStart(bool useWarmStart){ m_useWarmStart = useWarmStart; }

      /** Get the current maximum number of iterations performed by the solver. */
      virtual unsigned int getMaximumIterations(){ return m_maxIter; }
      /** Set the current maximum number of iterations performed by the solver. */
      virtual bool setMaximumIterations(unsigned int maxIter);


      /** Get the maximum time allowed to solve a problem. */
      virtual double getMaximumTime(){ return m_maxTime; }
      /** Set the maximum time allowed to solve a problem. */
      virtual bool setMaximumTime(double seconds);

    protected:

      std::string           m_name;
      bool                  m_useWarmStart;   // true if the solver is allowed to warm start
      int                   m_maxIter;        // max number of iterations
      double                m_maxTime;        // max time to solve the HQP [s]
      HQPOutput             m_output;

      unsigned int m_neq;  /// number of equality constraints
      unsigned int m_nin;  /// number of inequality constraints
      unsigned int m_n;    /// number of variables

      QPDataTpl<double> m_qpData;

    };

  }
}

#endif // ifndef __invdyn_solvers_hqp_base_hpp__
