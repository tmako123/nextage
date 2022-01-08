/*******************************************************
 * Copyright(c) 2018-2022, tmako123
 * All rights reserved.
 *
 * This file is distributed under the GNU Lesser General Public License v3.0.
 * The complete license agreement can be obtained at :
 * http://www.gnu.org/licenses/lgpl-3.0.html
*******************************************************/

#include <Eigen/Core>
#include <iostream>
#include <random>
#include <vector>

#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

//   F = 1/2 (f1^2 + f2^2 + f3^2 + f4^2)
//   f1 = x1 + 10 * x2
//   f2 = sqrt(5) * (x3 - x4)
//   f3 = (x2 - 2 * x3)^2
//   f4 = sqrt(10) * (x1 - x4)^2
class VertexParams : public g2o::BaseVertex<4, Eigen::Vector4d> {
public:
    VertexParams() {}

    virtual bool read(std::istream& /*is*/) { return false; }
    virtual bool write(std::ostream& /*os*/) const { return false; }
    virtual void setToOriginImpl() {}
    virtual void oplusImpl(const double* update)
    {
        Eigen::Vector4d::ConstMapType v(update);
        _estimate += v;
    }
};

class EdgePointOnPowellFunction
    : public g2o::BaseUnaryEdge<4, double, VertexParams> {
public:
    EdgePointOnPowellFunction() {}
    virtual bool read(std::istream& /*is*/) { return false; }
    virtual bool write(std::ostream& /*os*/) const { return false; }
    void computeError()
    {
        const VertexParams* vertices = static_cast<const VertexParams*>(_vertices[0]);
        const double a = vertices->estimate()[0];
        const double b = vertices->estimate()[1];
        const double c = vertices->estimate()[2];
        const double d = vertices->estimate()[3];
        _error[0] = a + 10.0 * b;
        _error[1] = sqrt(5.0) * (c - d);
        _error[2] = (b - 2 * c) * (b - 2 * c);
        _error[3] = sqrt(10) * (a - d) * (a - d);
    }
};

int main(int argc, char** argv)
{
    int maxIterations = 10;
    bool verbose = false;

    // problem y = 0.5x^2 + 1.2x + 1
    double a = 0.0, b = 0.0, c = 0.0, d = 0.0; //ground truth
    double a_ = 3.0, b_ = 1.0, c_ = 2.0, d_ = 1.0; //initial value

    // setup the solver
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    auto linearSolver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverPL<-1, -1>::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverPL<-1, -1>>(std::move(linearSolver)));
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // build the optimization problem given the points
    // 1. add the parameter vertex
    VertexParams* params = new VertexParams();
    params->setId(0);
    params->setEstimate(Eigen::Vector4d(a_, b_, c_, d_));
    optimizer.addVertex(params);

    // 2. add the measured to be on the linear function
    EdgePointOnPowellFunction* e = new EdgePointOnPowellFunction;
    e->setInformation(Eigen::Matrix4d::Identity());
    e->setVertex(0, params);
    optimizer.addEdge(e);

    // perform the optimization
    optimizer.initializeOptimization();
    optimizer.setVerbose(verbose);
    optimizer.optimize(maxIterations);

    if (verbose) {
        std::cout << std::endl;
    }

    // print out the result
    std::cout << "Target Function" << std::endl;
    std::cout << "f1 = x1 + 10*x2" << std::endl;
    std::cout << "f2 = sqrt(5) * (x3 - x4)" << std::endl;
    std::cout << "f3 = (x2 - 2*x3)^2" << std::endl;
    std::cout << "f4 = sqrt(10) * (x1 - x4)^2" << std::endl;
    std::cout << "Iterative least squares solution" << std::endl;
    std::cout << "a  = " << a_ << " -> " << params->estimate()(0) << " for " << a << std::endl;
    std::cout << "b  = " << b_ << " -> " << params->estimate()(1) << " for " << b << std::endl;
    std::cout << "c  = " << c_ << " -> " << params->estimate()(2) << " for " << c << std::endl;
    std::cout << "d  = " << d_ << " -> " << params->estimate()(3) << " for " << d << std::endl;
    return 0;
}