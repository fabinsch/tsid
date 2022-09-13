//
// Copyright (c) 2018 CNRS
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

#ifndef __tsid_python_util_container_hpp__
#define __tsid_python_util_container_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/solvers/fwd.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-inequality.hpp"
#include "tsid/math/constraint-bound.hpp"

using namespace std;
namespace tsid
{
  namespace python
  {         
    typedef solvers::ConstraintLevel ConstraintLevel;
    typedef solvers::HQPData HQPData;

    class ConstraintLevels
    {
    public:
        ConstraintLevels(){
        // ConstraintLevels():m_test_aligned_pair(1.0, make_shared<math::ConstraintInequality>("test")){
            // m_test.push_back(1.0);
            // m_test.push_back(1.0);
            // m_test_aligned.push_back(1.0);
            // m_test_aligned.push_back(1.0);
            // m_test_aligned.push_back(1.0);
            // double test_d = 1.0;
            // math::ConstraintInequality c = math::ConstraintInequality::ConstraintInequality('test');
            // std::shared_ptr<math::ConstraintBase> > test_c = make_shared<math::ConstraintBase>(c);
            // m_test_aligned_pair = solvers::make_pair<double, std::shared_ptr<math::ConstraintBase> >(test_d, test_c);
            // m_test_aligned_pointer.push_back(std::make_shared<double>(1.0));
            }
        ~ConstraintLevels(){}

        inline void print (){
            stringstream ss;
            for(ConstraintLevel::const_iterator iit=m_std_const.begin(); iit!=m_std_const.end(); iit++)
            {
                auto c = iit->second;
                ss<<" - "<<c->name()<<": w="<<iit->first<<", ";
                if(c->isEquality())
                    ss<<"equality, ";
                else if(c->isInequality())
                    ss<<"inequality, ";
                else
                    ss<<"bound, ";
                ss<<c->rows()<<"x"<<c->cols()<<endl;
            }
            cout << ss.str() << endl;
        }
        // const solvers::aligned_pair<double, std::shared_ptr<math::ConstraintBase> > & get_test_aligned_pair () const {
        //     return m_test_aligned_pair;
        // }
        ConstraintLevel& get (){
            return m_std_const;
        }

        solvers::aligned_pair<double, std::shared_ptr<math::ConstraintBase> > & get_(const size_t index) {
            if (index >= size(m_std_const))
                throw std::runtime_error("index out of bounds");
            return m_std_const[index];
        }


        // void get_constraint_vector (){
        //     stringstream ss;

        //     std::vector<math::ConstraintInequality> ineqVec;
        //     std::vector<std::shared_ptr<math::ConstraintEquality>> eqVec;
        //     std::vector<math::ConstraintBase> baseVec;
        //     for(ConstraintLevel::const_iterator iit=m_std_const.begin(); iit!=m_std_const.end(); iit++)
        //     {
        //         auto c = iit->second;
        //         // ss<<" - "<<c->name()<<": w="<<iit->first<<", ";
        //         if(c->isEquality()){
                    
        //         }
            
        //             // ss<<"equality, ";

        //             // eqVec.push_back(static_cast<std::shared_ptr<math::ConstraintEquality>>(c));
        //             // baseVec.push_back(static_cast<std::shared_ptr<math::ConstraintEquality>>(c));
        //             baseVec.push_back((c[0]));
        //         // else if(c->isInequality())
        //         //     // ss<<"inequality, ";
        //         //     ineqVec.push_back(*c);
        //         // else
        //         //     // ss<<"bound, ";
        //         // ss<<c->rows()<<"x"<<c->cols()<<endl;
        //     }
        //     // return cv;
        // }

        // const std::vector<double> & get_test () const {
        //     return m_test;
        // }

        // const pinocchio::container::aligned_vector<double> & get_test_aligned () const {
        //     return m_test_aligned;
        // }
    
        // const pinocchio::container::aligned_vector<std::shared_ptr<double> > & get_test_aligned_pointer () const {
        //     return m_test_aligned_pointer;
        // }

        inline void append_eq (double num, std::shared_ptr<math::ConstraintEquality> i){
           m_std_const.push_back(solvers::make_pair<double, std::shared_ptr<math::ConstraintBase> >(num, i));
        }
        inline void append_ineq (double num, std::shared_ptr<math::ConstraintInequality> i){
           m_std_const.push_back(solvers::make_pair<double, std::shared_ptr<math::ConstraintBase> >(num, i));
        }
        inline void append_bound (double num, std::shared_ptr<math::ConstraintBound> i){
           m_std_const.push_back(solvers::make_pair<double, std::shared_ptr<math::ConstraintBase> >(num, i));
        }

        // bool operator==(const ConstraintLevels & other) const
        // { return true; }

    private:
        ConstraintLevel m_std_const;
    //     std::vector<double> m_test;
    //     pinocchio::container::aligned_vector<double> m_test_aligned;
    //     solvers::aligned_pair<double, std::shared_ptr<math::ConstraintBase> > m_test_aligned_pair;
    };

    class HQPDatas
    {
    public:
        HQPDatas(){}
        ~HQPDatas(){}

        inline void resize (size_t i) {
           m_std_hqp.resize(i);
        }

        inline void print () const {
            stringstream ss;
            unsigned int priority = 0;
            for(HQPData::const_iterator it=m_std_hqp.begin(); it!=m_std_hqp.end(); it++)
            {
                ss<<"Level "<< priority<<endl;
                for(ConstraintLevel::const_iterator iit=it->begin(); iit!=it->end(); iit++)
                {
                auto c = iit->second;
                ss<<" - "<<c->name()<<": w="<<iit->first<<", ";
                if(c->isEquality())
                    ss<<"equality, ";
                else if(c->isInequality())
                    ss<<"inequality, ";
                else
                    ss<<"bound, ";
                ss<<c->rows()<<"x"<<c->cols()<<endl;
                }
                priority++;
            }
             cout << ss.str() << endl;
        }
        // inline void append (ConstraintLevel cons){
        //     m_std_hqp.push_back(cons);
        // }
        inline void append_helper (ConstraintLevels* cons){
            m_std_hqp.push_back(cons->get());
        }

        inline HQPData get (){
            return m_std_hqp;
        }
        inline bool set (HQPData data){
            m_std_hqp = data;
            return true;
        }

        solvers::aligned_pair<double, std::shared_ptr<math::ConstraintBase> > & get_(const size_t idx0, const size_t idx1) {
            if (idx0 >= size(m_std_hqp))
                throw std::runtime_error("index out of bounds");
            if (idx1 >= size(m_std_hqp[idx0]))
                throw std::runtime_error("index out of bounds");
            return m_std_hqp[idx0][idx1];
        }
    
    private:
        HQPData m_std_hqp;
    };
  }
}


#endif // ifndef __tsid_python_util_container_hpp__