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

#include "tsid/bindings/python/constraint/constraint-base.hpp"
#include "tsid/bindings/python/constraint/expose-constraints.hpp"

namespace tsid
{
  namespace python
  {
    void exposeConstraintBase()
    {
      bp::class_<ConstraintBaseWrap, boost::noncopyable>("ConstraintBase", bp::init<const std::string &>(bp::args("self", "name")," Constructor from name."))
      .def(bp::init<const std::string &, const unsigned int, const unsigned int>(bp::args("self","name", "rows", "cols"),"Constructor from a name, cols, rows."))
      .def(bp::init<const std::string &, ConstRefMatrix>(bp::args("self", "name", "name"), "Constructor from name and matrix."))
      .def("name", &math::ConstraintBase::name, &ConstraintBaseWrap::default_name, bp::return_internal_reference<>())
      .def("rows", bp::pure_virtual(&math::ConstraintBase::rows))
      .def("cols", bp::pure_virtual(&math::ConstraintBase::cols))
      .def("resize", bp::pure_virtual(&math::ConstraintBase::resize))
      .def("isEquality", bp::pure_virtual(&math::ConstraintBase::isEquality))
      .def("isInequality", bp::pure_virtual(&math::ConstraintBase::isInequality))
      .def("isBound", bp::pure_virtual(&math::ConstraintBase::isBound))
      .def("matrix", (const Matrix & (math::ConstraintBase::*)() const) &math::ConstraintBase::matrix, (const Matrix & (math::ConstraintBase::*)() const) &ConstraintBaseWrap::default_matrix, bp::return_internal_reference<>())
      .def("vector", (const Vector & (math::ConstraintBase::*)() const) &math::ConstraintBase::vector, bp::return_internal_reference<>())
      .def("lowerBound", (const Vector & (math::ConstraintBase::*)() const) &math::ConstraintBase::lowerBound, bp::return_internal_reference<>())
      .def("upperBound", (const Vector & (math::ConstraintBase::*)() const) &math::ConstraintBase::upperBound, bp::return_internal_reference<>())
      .def("matrix", (Matrix & (math::ConstraintBase::*)()) &math::ConstraintBase::matrix, (Matrix & (math::ConstraintBase::*)()) &ConstraintBaseWrap::default_matrix, bp::return_internal_reference<>())
      .def("vector", (Vector & (math::ConstraintBase::*)()) &math::ConstraintBase::vector, bp::return_internal_reference<>())
      .def("lowerBound", (Vector & (math::ConstraintBase::*)()) &math::ConstraintBase::lowerBound, bp::return_internal_reference<>())
      .def("upperBound", (Vector & (math::ConstraintBase::*)()) &math::ConstraintBase::upperBound, bp::return_internal_reference<>())
      .def("setMatrix", &math::ConstraintBase::setMatrix, &ConstraintBaseWrap::default_setMatrix)
      .def("setVector", bp::pure_virtual(&math::ConstraintBase::setVector))
      .def("setLowerBound", bp::pure_virtual(&math::ConstraintBase::setLowerBound))
      .def("setUpperBound", bp::pure_virtual(&math::ConstraintBase::setUpperBound))
      .def("checkConstraint", bp::pure_virtual(&math::ConstraintBase::checkConstraint))
      ;
    }
  }
}
