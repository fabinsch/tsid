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

#include "tsid/solvers/solver-HQP-base.hpp"
#include "tsid/math/utils.hpp"

#include <iostream>

namespace tsid
{
  namespace solvers
  {
    using namespace math;

    std::string const SolverHQPBase::HQP_status_string[] = { "HQP_STATUS_OPTIMAL",
                                                  "HQP_STATUS_INFEASIBLE",
                                                  "HQP_STATUS_UNBOUNDED",
                                                  "HQP_STATUS_MAX_ITER_REACHED",
                                                  "HQP_STATUS_ERROR"};

    SolverHQPBase::SolverHQPBase(const std::string & name)
    {
      m_name = name;
      m_maxIter = 1000;
      m_maxTime = 100.0;
      m_useWarmStart = true;
      m_n = 0;
      m_neq = 0;
      m_nin = 0;
    }

    bool SolverHQPBase::setMaximumIterations(unsigned int maxIter)
    {
      if(maxIter==0)
        return false;
      m_maxIter = maxIter;
      return true;
    }

    bool SolverHQPBase::setMaximumTime(double seconds)
    {
      if(seconds<=0.0)
        return false;
      m_maxTime = seconds;
      return true;
    }

    // void SolverHQPBase::sendMsg(const std::string & s)
    // {
    //   std::cout<<"[SolverHQPBase."<<m_name<<"] "<<s<<std::endl;
    // }

    void SolverHQPBase::resizeIneq(unsigned int n, unsigned int nin)
    {
  #ifndef NDEBUG
      sendMsg("Resizing inequality constraints from "+ toString(m_nin)+" to "+toString(nin));
  #endif
      // m_qpData.CI.resize(2*nin, n);
      // m_qpData.ci0.resize(2*nin);
      m_qpData.CI.resize(nin, n);
      m_qpData.ci_lb.resize(nin);
      m_qpData.ci_ub.resize(nin);
    }

    bool SolverHQPBase::resize(unsigned int n, unsigned int neq, unsigned int nin)
    {
      const bool resizeVar = n!=m_n;
      const bool resizeEq = (resizeVar || neq!=m_neq );
      const bool resizeIn = (resizeVar || nin!=m_nin );

      if(resizeEq)
      {
    #ifndef NDEBUG
        sendMsg("Resizing equality constraints from "+toString(m_neq)+" to "+toString(neq));
    #endif
        m_qpData.CE.resize(neq, n);
        m_qpData.ce0.resize(neq);
      }
      if(resizeIn)
      {
        resizeIneq(n, nin);
      }
      if(resizeVar)
      {
    #ifndef NDEBUG
        sendMsg("Resizing Hessian from "+toString(m_n)+" to "+toString(n));
    #endif
        m_qpData.H.resize(n, n);
        m_qpData.g.resize(n);
        m_output.x.resize(n);
      }

      m_n = n;
      m_neq = neq;
      m_nin = nin;

      return (resizeVar || resizeEq || resizeIn);
    }

    void SolverHQPBase::setIneq(unsigned int & i_in, const std::shared_ptr<math::ConstraintBase> constr)
    {
      // std::cout << "{0} i_in " << i_in << std::endl;
      m_qpData.CI.middleRows(i_in, constr->rows()) = constr->matrix();
      m_qpData.ci_lb.segment(i_in, constr->rows()) = constr->lowerBound();
      m_qpData.ci_ub.segment(i_in, constr->rows()) = -constr->upperBound();
      i_in += constr->rows();
    }

    void SolverHQPBase::setBounds(unsigned int & i_in, const std::shared_ptr<math::ConstraintBase> constr)
    {
      // std::cout << "{0} i_in " << i_in << std::endl;
      m_qpData.CI.middleRows(i_in, constr->rows()).setIdentity();
      m_qpData.ci_lb.segment(i_in, constr->rows()) = constr->lowerBound();
      m_qpData.ci_ub.segment(i_in, constr->rows()) = -constr->upperBound();
      i_in += constr->rows();
    }

    void SolverHQPBase::retrieveQPData(const HQPData & problemData, const bool hessianRegularization)
    {

      // std::cout << "[0] retrievePData" << std::endl;
      if(problemData.size() > 2)
          {
            assert(false && "Solver not implemented for more than 2 hierarchical levels.");
          }
          
          // Compute the constraint matrix sizes
          unsigned int neq = 0, nin = 0;
          const ConstraintLevel & cl0 = problemData[0];
          // std::cout << "cl0.size() " << cl0.size() << std::endl;

          if(cl0.size()>0)
          {
            const unsigned int n = cl0[0].second->cols();
            for(ConstraintLevel::const_iterator it=cl0.begin(); it!=cl0.end(); it++)
            {
              auto constr = it->second;
              assert(n==constr->cols());
              if(constr->isEquality())
                neq += constr->rows();
              else
                nin += constr->rows();
            }
            // If necessary, resize the constraint matrices
            resize(n, neq, nin);

            // std::cout << "m_qpData.CE.rows() " << m_qpData.CE.rows() << std::endl;
            // std::cout << "m_qpData.ce0.rows() " << m_qpData.ce0.rows() << std::endl;
            // std::cout << "m_qpData.CI.rows() " << m_qpData.CI.rows() << std::endl;
            
            unsigned int i_eq = 0, i_in = 0;
            for(ConstraintLevel::const_iterator it=cl0.begin(); it!=cl0.end(); it++)
            {
              // std::cout << " ********** " << std::endl;

              auto constr = it->second;
              if(constr->isEquality())
              {
                // std::cout << "i_eq " << i_eq << std::endl;
                // std::cout << "constr->rows() " << constr->rows() << std::endl;
                // std::cout << "constr->matrix() " << constr->matrix() << std::endl;
                // std::cout << "constr->vector() " << constr->vector() << std::endl;

                m_qpData.CE.middleRows(i_eq, constr->rows()) = constr->matrix();
                m_qpData.ce0.segment(i_eq, constr->rows())   = -constr->vector();
                i_eq += constr->rows();
                // std::cout << "done isEq " << std::endl;

              }
              else if(constr->isInequality())
              {
                setIneq(i_in, constr);
              }
              else if(constr->isBound())
              {
                setBounds(i_in, constr);
              }
            }
          }
          else
            resize(m_n, neq, nin);
          
          EIGEN_MALLOC_NOT_ALLOWED;

          // Compute the cost 
          if(problemData.size() > 1)
          {
            // std::cout << "Computing H and g " << std::endl;
            const ConstraintLevel & cl1 = problemData[1];
            m_qpData.H.setZero();
            m_qpData.g.setZero();
            
            for(ConstraintLevel::const_iterator it=cl1.begin(); it!=cl1.end(); it++)
            {
              const double & w = it->first;
              auto constr = it->second;
              if(!constr->isEquality())
                assert(false && "Inequalities in the cost function are not implemented yet");
              
              EIGEN_MALLOC_ALLOWED;
              m_qpData.H.noalias() += w*constr->matrix().transpose()*constr->matrix();
              EIGEN_MALLOC_NOT_ALLOWED;
              
              m_qpData.g.noalias() -= w*constr->matrix().transpose()*constr->vector();
            }
            
            if (hessianRegularization)
            {
              double m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION);
              m_qpData.H.diagonal().array() += m_hessian_regularization;
            }
          }
      // std::cout << "[1] retrievePData" << std::endl;
    }
  }
}
