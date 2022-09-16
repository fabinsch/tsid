//
// Copyright (c) 2022 INRIA
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


#ifndef __tsid_python_constraint_base_hpp__
#define __tsid_python_constraint_base_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/math/constraint-base.hpp"

namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;

    typedef math::Matrix Matrix;
    typedef math::Vector Vector;
    typedef math::RefVector RefVector;
    typedef math::ConstRefVector ConstRefVector;
    typedef math::ConstRefMatrix ConstRefMatrix;
  
    struct ConstraintBaseWrap : math::ConstraintBase, bp::wrapper<math::ConstraintBase>
    {
      ConstraintBaseWrap(const std::string & name) : math::ConstraintBase(name) {}
      ConstraintBaseWrap(const std::string & name, const unsigned int rows, const unsigned int cols) : math::ConstraintBase(name, rows, cols) {}
      ConstraintBaseWrap(const std::string & name, ConstRefMatrix A) : math::ConstraintBase(name, A) {}

      const std::string & name() const
      {
        if (bp::override name = this->get_override("name"))
            return name();  
        return math::ConstraintBase::name();
      }
      const std::string & default_name() const { return this->math::ConstraintBase::name(); }
      
      unsigned int rows() const
      {
        return this->get_override("rows")();
      }
      
      unsigned int cols() const
      {
        return this->get_override("cols")();
      }

      void resize(const unsigned int r, const unsigned int c)
      {
        this->get_override("resize")(r, c);
      }
      
      bool isEquality() const
      {
        return this->get_override("isEquality")();
      }
 
      bool isInequality() const
      {
        return this->get_override("isInequality")();
      }

      bool isBound() const
      {
        return this->get_override("isBound")();
      }

      const Matrix & matrix() const
      {
        if (bp::override matrix = this->get_override("matrix"))
            return matrix();
        return math::ConstraintBase::matrix();
      }
      const Matrix & default_matrix() const { return this->math::ConstraintBase::matrix(); }


      const Vector & vector() const
      {
        return this->get_override("vector")();
      }

      const Vector & lowerBound() const
      {
        return this->get_override("lowerBound")();
      }

      const Vector & upperBound() const
      {
        return this->get_override("upperBound")();
      }

      Matrix & matrix()
      {
        if (bp::override matrix = this->get_override("matrix"))
            return matrix();
        return math::ConstraintBase::matrix();
      }
      Matrix & default_matrix() { return this->math::ConstraintBase::matrix(); }

      Vector & vector()
      {
        return this->get_override("vector")();
      }

      Vector & lowerBound()
      {
        return this->get_override("lowerBound")();
      }

      Vector & upperBound()
      {
        return this->get_override("upperBound")();
      }

      bool setMatrix(ConstRefMatrix A)
      {
        if (bp::override setMatrix = this->get_override("setMatrix"))
            return setMatrix(A);
        return math::ConstraintBase::setMatrix(A);
      }
      bool default_setMatrix(ConstRefMatrix A) { return this->math::ConstraintBase::setMatrix(A); }

      bool setVector(ConstRefVector b)
      {
        return this->get_override("setVector")();
      }

      bool setLowerBound(ConstRefVector lb)
      {
        return this->get_override("setLowerBound")();
      }

      bool setUpperBound(ConstRefVector ub)
      {
        return this->get_override("setUpperBound")();
      }

      bool checkConstraint(ConstRefVector x, double tol=1e-6) const
      {
        return this->get_override("checkConstraint")();
      }

    };
  }
}


#endif // ifndef __tsid_python_constraint_base_hpp__