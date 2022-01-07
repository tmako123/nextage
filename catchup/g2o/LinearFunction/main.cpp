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

//the params, a and b for ax + b
class VertexParams : public g2o::BaseVertex<2, Eigen::Vector2d> {
public:
    VertexParams() {}

    virtual bool read(std::istream& /*is*/) { return false; }
    virtual bool write(std::ostream& /*os*/) const { return false; }
    virtual void setToOriginImpl() {}
    virtual void oplusImpl(const double* update)
    {
        Eigen::Vector2d::ConstMapType v(update);
        _estimate += v;
    }
};

class EdgePointOnLinearFunction
    : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexParams> {
public:
    EdgePointOnLinearFunction() {}
    virtual bool read(std::istream& /*is*/) { return false; }
    virtual bool write(std::ostream& /*os*/) const { return false; }
    void computeError()
    {
        const VertexParams* vertices = static_cast<const VertexParams*>(_vertices[0]);
        const double a = vertices->estimate()[0];
        const double b = vertices->estimate()[1];
        const double x = measurement()[0];
        const double y = a * x + b;
        _error[0] = y - measurement()[1];
    }
};

int main(int argc, char** argv)
{
    int maxIterations = 10;
    bool verbose = false;

    // problem y = 0.5 x + 1
    double a = 0.5, b = 1.0; //ground truth
    double a_ = 0.0, b_ = 0.0; //initial value
    double mu = 0., sigma = 0.3; // measurement variance

    // create masurements
    std::vector<Eigen::Vector2d> xy, xy_;
    std::normal_distribution<> dist(mu, sigma);
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());

    double minX = -5.0, maxX = 5.0, resoX = 0.1;
    int numX = (maxX - minX) / resoX + 1;

    for (int i = 0; i < numX; i++) {
        double x = minX + resoX * i;
        double y = a * x + b;
        xy.push_back(Eigen::Vector2d(x, y));
        xy_.push_back(Eigen::Vector2d(x + dist(engine), y + dist(engine)));
    }

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
    params->setEstimate(Eigen::Vector2d(a_, b_));
    optimizer.addVertex(params);

    // 2. add the measured to be on the linear function
    for (int i = 0; i < xy_.size(); ++i) {
        EdgePointOnLinearFunction* e = new EdgePointOnLinearFunction;
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        e->setVertex(0, params);
        e->setMeasurement(xy_[i]);
        optimizer.addEdge(e);
    }

    // perform the optimization
    optimizer.initializeOptimization();
    optimizer.setVerbose(verbose);
    optimizer.optimize(maxIterations);

    if (verbose) {
        std::cout << std::endl;
    }

    // print out the result
    std::cout << "Target Function" << std::endl;
    std::cout << "ax + b" << std::endl;
    std::cout << "Iterative least squares solution" << std::endl;
    std::cout << "a  = " << a_ << " -> " << params->estimate()(0) << " for " << a << std::endl;
    std::cout << "b  = " << b_ << " -> " << params->estimate()(1) << " for " << b << std::endl;
    return 0;
}