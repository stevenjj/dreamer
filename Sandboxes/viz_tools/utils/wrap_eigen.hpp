/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (C) 2010 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file sejong/wrap_eigen.hpp
   \author Roland Philippsen
*/

#ifndef SEJONG_WRAP_EIGEN_HPP
#define SEJONG_WRAP_EIGEN_HPP

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>
#include <list>
#include <string>
#include <cmath>
#include <ctime>

namespace sejong {
    typedef Eigen::Transform<double, 3, Eigen::Affine> Transform;
    typedef Eigen::Translation3d Translation;
    typedef Eigen::Quaternion<double> Quaternion;
    typedef Eigen::Matrix<double,3,1> Vect3;
    typedef Eigen::Matrix<double,4,1> Vect4;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;
    // Euler ange (Yaw, Pitch, Roll) to Quaternion
    void convert(double yaw, double pitch, double roll, sejong::Quaternion& to);
    // Quaternion to Euler ZYX
    void convert(const sejong::Quaternion& from, double & yaw, double & pitch, double & roll);
    // Quaternion to so(3)
    void convert(sejong::Quaternion const & from, sejong::Vector & to);
    
    // so(3) to Quaternion
    void convert(sejong::Vector const & from, sejong::Quaternion & to);
    
    void convert(sejong::Vector const & from, std::vector<double> & to);
    void convert(std::vector<double> const & from, sejong::Vector & to);
    void convert(double const * from, size_t length, sejong::Vector & to);

    Quaternion QuatMultiply(const Quaternion & q1, const Quaternion & q2);

    bool compare(sejong::Matrix const & lhs, sejong::Matrix const & rhs, double precision);
    bool compare(sejong::Quaternion const & lhs, sejong::Quaternion const & rhs, double precision);

    double _bind_half_pi(double);
}

#endif
