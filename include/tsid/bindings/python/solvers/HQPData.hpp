//
// Copyright (c) 2018 CNRS, NYU, MPI Tübingen
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

#ifndef __tsid_python_test_hpp__
#define __tsid_python_test_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/bindings/python/utils/container.hpp"

namespace tsid
{
  namespace python
  {    

    // // const std::vector<math::ConstraintBase> get_data_custom_ (ConstraintLevel t) {
    // // void get_data_custom_ (const boost::python::object self) {
    // void get_data_custom_ (const boost::python::object self, const ConstraintLevel & cl) {
    //   // ConstraintLevel& thisref = boost::python::extract<ConstraintLevel&>(self)();
    //   // solvers::aligned_pair ap = thisref[0];
    //   // std::cout << "ap.first" << ap.first << std::endl;
    //   std::cout << "cl" <<  std::endl;
      
    //   // std::vector<math::ConstraintBase> a;
    //   // return a;
    // }

    namespace bp = boost::python;
    
    template<typename T>
    struct ConstPythonVisitor
    : public boost::python::def_visitor< ConstPythonVisitor<T> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default Constructor"))
        .def("print_all", &T::print)
        .def("append", &T::append_eq, bp::arg("data"))
        .def("append", &T::append_ineq, bp::arg("data"))
        .def("append", &T::append_bound, bp::arg("data"))  
        .def("get", &T::get_, bp::return_internal_reference<>())  
        // .def("get", &T::get_, bp::return_value_policy<bp::copy_const_reference>())  

        // .def("get_data", &T::get_data, bp::return_internal_reference<>())  
        // .def("get_data", &get_data, bp::return_internal_reference<>())  
        // .def("get_data_custom", get_data_custom_, bp::arg("self"))
        // .def("get_data_custom", get_data_custom_)
        // .def("get_constraint_vector", &T::get_constraint_vector)
        // .def("get_test", &T::get_test, bp::return_internal_reference<>())  
        // .def("get_test_aligned", &T::get_test_aligned, bp::return_internal_reference<>())  
        // .def("get_test_aligned_pair", &T::get_test_aligned_pair, bp::return_internal_reference<>())  
        // .def("get_test_aligned_pair", &T::get_test_aligned_pair, bp::return_value_policy<bp::copy_const_reference>())  
        // .def("get", &T::get)  
        ;
      }

       
      static void expose(const std::string & class_name)
      {
        std::string doc = "ConstraintLevel info.";
        bp::class_<T>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(ConstPythonVisitor<T>());       
      }
    };

    template<typename T>
    struct HQPPythonVisitor
    : public boost::python::def_visitor< HQPPythonVisitor<T> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default Constructor"))
        .def("print_all", &T::print)
        .def("resize", &T::resize, bp::arg("i"))
        .def("append", &T::append_helper, bp::arg("constraintLevel"))  
        .def("get", &T::get_, bp::args("idx0", "idx1"), bp::return_internal_reference<>())  
        ;
      }
       
      static void expose(const std::string & class_name)
      {
        std::string doc = "HQPdata info.";
        bp::class_<T>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(HQPPythonVisitor<T>());       
      }
    };

    template<typename T>
    struct PairPythonVisitor
    : public boost::python::def_visitor< PairPythonVisitor<T> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<double, std::shared_ptr<math::ConstraintBase> >("Constructor"))
        .def_readonly("first", &T::first)
        .def_readonly("second", &T::second)
        // .def("get_ineq", &T::get_ineq, bp::return_internal_reference<>())  

        ;
      }

      static void expose(const std::string & class_name)
      {
        std::string doc = "Aligned pair.";
        bp::class_<T>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(PairPythonVisitor<T>());       
      }
    };

  }
}


#endif // ifndef __tsid_python_test_hpp__
