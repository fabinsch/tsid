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

#include "tsid/solvers/solver-proxqp.hpp"
#include "tsid/math/utils.hpp"
#include "tsid/utils/stop-watch.hpp"

// using namespace eiquadprog::solvers;

namespace tsid
{
  namespace solvers
  {
    
    using namespace math;
    SolverProxQP::SolverProxQP(const std::string & name):
    SolverHQPBase(name),
    m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION),
    m_solver(0, 0, 0),
    m_output()
    {
      // m_solver = dense::QP<double>(0, 0, 0);
    }
    
    void SolverProxQP::sendMsg(const std::string & s)
    {
      std::cout<<"[SolverProxQP."<<m_name<<"] "<<s<<std::endl;
    }
    
    // const bool resize(unsigned int n, unsigned int neq, unsigned int nin)
    // {
    //   bool resized = SolverHQPBase::resize(n, neq, nin);  // not possible bc no object of type parent
    //   if (resized)
    //   {
    //     m_solver = dense::QP<double> Qp{ n, neq, nin }; // creating QP object
    //   }
    // }

    const HQPGenericOutput & SolverProxQP::solve(const HQPData & problemData)
    {

      SolverProxQP::retrieveQPData(problemData);

      // just if resized
      m_solver = dense::QP<double>(m_n, m_neq, m_nin);

      START_PROFILER("PROFILE_PROXQP_SOLUTION");
      //  min 0.5 * x G x + g0 x
      //  s.t.
      //  CE x + ce0 = 0
      //  ci_lb <= CI x <= ci_ub
      EIGEN_MALLOC_ALLOWED
      // std::cout << "here" << std::endl;
      // std::cout << "m_qpData.H: " << m_qpData.H << std::endl;
      // std::cout << "m_qpData.g: " << m_qpData.g << std::endl;
      // std::cout << "m_qpData.CE: " << m_qpData.CE << std::endl;
      // std::cout << "m_qpData.ce0: " << m_qpData.ce0 << std::endl;
      // std::cout << "m_qpData.CI: " << m_qpData.CI << std::endl;
      // std::cout << "m_qpData.ci0: " << m_qpData.ci0 << std::endl;
      m_solver.init(SolverHQPBase::m_qpData.H, SolverHQPBase::m_qpData.g,
                     SolverHQPBase::m_qpData.CE, SolverHQPBase::m_qpData.ce0,
                     m_qpData.CI, m_qpData.ci_ub, m_qpData.ci_lb);
      m_solver.solve();
      STOP_PROFILER("PROFILE_PROXQP_SOLUTION");
      
      QPSolverOutput status = m_solver.results.info.status;
      
      if(status == QPSolverOutput::PROXQP_SOLVED)
      {
        m_output.results = m_solver.results;
#ifndef NDEBUG
        const Vector & x = m_solver.results.x;

        const ConstraintLevel & cl0 = problemData[0];

        if(cl0.size()>0)
        {
          for(ConstraintLevel::const_iterator it=cl0.begin(); it!=cl0.end(); it++)
          {
            auto constr = it->second;
            if(constr->checkConstraint(x)==false)
            {
              // m_output.status = HQP_STATUS_ERROR;
              if(constr->isEquality())
              {
                sendMsg("Equality "+constr->name()+" violated: "+
                        toString((constr->matrix()*x-constr->vector()).norm()));
              }
              else if(constr->isInequality())
              {
                sendMsg("Inequality "+constr->name()+" violated: "+
                        toString((constr->matrix()*x-constr->lowerBound()).minCoeff())+"\n"+
                        toString((constr->upperBound()-constr->matrix()*x).minCoeff()));
              }
              else if(constr->isBound())
              {
                sendMsg("Bound "+constr->name()+" violated: "+
                        toString((x-constr->lowerBound()).minCoeff())+"\n"+
                        toString((constr->upperBound()-x).minCoeff()));
              }
            }
          }
        }
#endif
      }
      // else if(status==EIQUADPROG_FAST_UNBOUNDED)
      //   m_output.status = HQP_STATUS_INFEASIBLE;
      // else if(status==EIQUADPROG_FAST_MAX_ITER_REACHED)
      //   m_output.status = HQP_STATUS_MAX_ITER_REACHED;
      // else if(status==EIQUADPROG_FAST_REDUNDANT_EQUALITIES)
      //   m_output.status = HQP_STATUS_ERROR;
      
      return m_output;
    }
    
    // double SolverProxQP::getObjectiveValue()
    // {
    //   return m_solver.getObjValue();
    // }
    
    // bool SolverProxQP::setMaximumIterations(unsigned int maxIter)
    // {
    //   SolverHQPBase::setMaximumIterations(maxIter);
    //   return m_solver.setMaxIter(maxIter);
    // }
  }
}


