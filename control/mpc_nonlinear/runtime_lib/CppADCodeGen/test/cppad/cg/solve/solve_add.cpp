/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */
#include "CppADCGSolveTest.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGSolveTest, SolveAdd) {
    // independent variable vector
    std::vector<ADCGD> x(2);
    x[0] = -1.5;
    x[1] = -0.5;
    Independent(x);

    // dependent variable vector
    std::vector<ADCGD> y(3);

    // model
    y[0] = x[0] + x[1]; // AD<double> + AD<double>
    y[1] = y[0] + 1.; // AD<double> + double
    y[2] = 1. + y[1]; // double + AD<double>

    // create f: x -> y
    ADFun<CGD> fun(x, y);

    test_solve(fun, 2, 1, x);
}
