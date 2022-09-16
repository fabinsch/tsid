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

#include "tsid/solvers/solver-HQP-eiquadprog-fast.hpp"
#include "tsid/math/utils.hpp"
#include "eiquadprog/eiquadprog-fast.hpp"
#include "tsid/utils/stop-watch.hpp"

//#define PROFILE_EIQUADPROG_FAST

using namespace eiquadprog::solvers;

namespace tsid
{
  namespace solvers
  {
    
    using namespace math;
    SolverHQuadProgFast::SolverHQuadProgFast(const std::string & name):
    SolverHQPBase(name),
    m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION)
    {}
    
    void SolverHQuadProgFast::sendMsg(const std::string & s)
    {
      std::cout<<"[SolverHQuadProgFast."<<m_name<<"] "<<s<<std::endl;
    }
    
    void SolverHQuadProgFast::resizeIneq(unsigned int n, unsigned int nin)
    {
#ifndef NDEBUG
      sendMsg("Resizing inequality constraints from "+toString(m_nin)+" to "+toString(nin));
#endif
      m_qpData.CI.resize(2*nin, n);
      m_qpData.ci0.resize(2*nin);
    }

    void SolverHQuadProgFast::setIneq(unsigned int & i_in, const std::shared_ptr<math::ConstraintBase> constr)
    {
      // std::cout << "{0} i_in " << i_in << std::endl;
      m_qpData.CI.middleRows(i_in, constr->rows()) = constr->matrix();
      m_qpData.ci0.segment(i_in, constr->rows())   = -constr->lowerBound();
      i_in += constr->rows();
      // std::cout << "{1} i_in " << i_in << std::endl;
      m_qpData.CI.middleRows(i_in, constr->rows()) = -constr->matrix();
      m_qpData.ci0.segment(i_in, constr->rows())   = constr->upperBound();
      i_in += constr->rows();
    }

    void SolverHQuadProgFast::setBounds(unsigned int & i_in, const std::shared_ptr<math::ConstraintBase> constr)
    {
      m_qpData.CI.middleRows(i_in, constr->rows()).setIdentity();
      m_qpData.ci0.segment(i_in, constr->rows())   = -constr->lowerBound();
      i_in += constr->rows();
      m_qpData.CI.middleRows(i_in, constr->rows()) = -Matrix::Identity(m_n, m_n);
      m_qpData.ci0.segment(i_in, constr->rows())   = constr->upperBound();
      i_in += constr->rows();
    }
    
    const HQPGenericOutput & SolverHQuadProgFast::solve(const HQPData & problemData)
    {

      SolverHQuadProgFast::retrieveQPData(problemData)

      START_PROFILER_EIQUADPROG_FAST(PROFILE_EIQUADPROG_SOLUTION);
      //  min 0.5 * x G x + g0 x
      //  s.t.
      //  CE x + ce0 = 0
      //  CI x + ci0 >= 0
      EIGEN_MALLOC_ALLOWED
      // std::cout << "here" << std::endl;
      // std::cout << "m_qpData.H: " << m_qpData.H << std::endl;
      // std::cout << "m_qpData.g: " << m_qpData.g << std::endl;
      // std::cout << "m_qpData.CE: " << m_qpData.CE << std::endl;
      // std::cout << "m_qpData.ce0: " << m_qpData.ce0 << std::endl;
      // std::cout << "m_qpData.CI: " << m_qpData.CI << std::endl;
      // std::cout << "m_qpData.ci0: " << m_qpData.ci0 << std::endl;

      eiquadprog::solvers::EiquadprogFast_status
          status = m_solver.solve_quadprog(SolverHQPBase::m_qpData.H, SolverHQPBase::m_qpData.g,
                                           SolverHQPBase::m_qpData.CE, SolverHQPBase::m_qpData.ce0,
                                           m_qpData.CI, m_qpData.ci0,
                                           m_output.x);
    
      STOP_PROFILER_EIQUADPROG_FAST(PROFILE_EIQUADPROG_SOLUTION);
      
      
      if(status == EIQUADPROG_FAST_OPTIMAL)
      {
        m_output.status = HQP_STATUS_OPTIMAL;
        m_output.lambda = m_solver.getLagrangeMultipliers();
        m_output.iterations = m_solver.getIteratios();
        //    m_output.activeSet = m_solver.getActiveSet().tail(2*m_nin).head(m_solver.getActiveSetSize()-m_neq);
        m_output.activeSet = m_solver.getActiveSet().segment(m_neq, m_solver.getActiveSetSize()-m_neq);
#ifndef NDEBUG
        const Vector & x = m_output.x;

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
      else if(status==EIQUADPROG_FAST_UNBOUNDED)
        m_output.status = HQP_STATUS_INFEASIBLE;
      else if(status==EIQUADPROG_FAST_MAX_ITER_REACHED)
        m_output.status = HQP_STATUS_MAX_ITER_REACHED;
      else if(status==EIQUADPROG_FAST_REDUNDANT_EQUALITIES)
        m_output.status = HQP_STATUS_ERROR;
      
      return m_output;
    }
    
    double SolverHQuadProgFast::getObjectiveValue()
    {
      return m_solver.getObjValue();
    }
    
    bool SolverHQuadProgFast::setMaximumIterations(unsigned int maxIter)
    {
      SolverHQPBase::setMaximumIterations(maxIter);
      return m_solver.setMaxIter(maxIter);
    }
  }
}


