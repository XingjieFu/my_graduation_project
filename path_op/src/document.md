**C++关键字inline**：避免了频繁调用函数对栈内存重复开辟所带来的消耗。在 c/c++ 中，为了解决一些频繁调用的小函数大量消耗栈空间（栈内存）的问题，特别的引入了 **inline** 修饰符，表示为内联函数。栈空间就是指放置程序的局部数据（也就是函数内数据）的内存空间。

海森矩阵：Hessian，存储了函数的二阶导数信息。

C++中Eigen用法：row，行。

**SparseMatrix，稀疏矩阵**，矩阵中非零元素的个数远远小于矩阵元素的总数，并且非零元素的分布没有规律，通常认为矩阵中非零元素的总数比上矩阵所有元素总数的值小于等于0.05时，则称该矩阵为稀疏矩阵。

const：首先作用于左边，如果左边没有东西，就作用于右边。

const int 

int const

const int* const

int* const：指针本身不可改变，常量指针。（顶层）

const int* ：指针指向的内容不可改变（底层）

Eigen用法总结：https://blog.csdn.net/augusdi/article/details/12907341。

complex指的是复数。

b的转置和a相乘代码如下：

```C++
 b.transpose()*a 
```

引入松弛变量的意义：1.引入松弛变量后的问题与原问题等价，最优解保持不变。2.通过引入松弛变量可以将原问题可行域扩展到更大的空间。3.等式约束在优化方面有更好的性质。

二次规划问题：https://www.cnblogs.com/zhjblogs/p/14958078.html

速度规划的优化问题描述为：min ReferenceCost(x) + AccCost(x) + JerkCost(x) + OmegaCost(x) + EpsilonCost(ϵ).

**二次型**： Rn 上的一个二次型是一个定义在 Rn 上的函数，表达式为 Q(x)=xTAx ，其中 A 是一个 n×n **对称矩阵**（一定要注意 A 是对称矩阵，不是一个随便的矩阵），矩阵 A 称为**关于二次型的矩阵（二次型矩阵）**

## OSQP学习

**1.相关类**

`osqp-eigen`主要是构造了三个类，Data类、Setting类、Solver类：

- Data类：对C语言OSQP中的OSQPData(struct)进行包装
- Setting类：对C语言OSQP中的OSQPSetting(struct)进行包装
- Solver类：对C语言OSQP中的OSQPWorkspace(struct)进行包装

**2.求解过程**

- 1.创建求解器Solver的实例
- 2.设置Setting参数
- 3.设置Data参数
- 4.初始化求解器Solver
- 5.求解优化问题
- 6.提取最优解

OSQP简单解方程例子：

```c++
// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>
#include <iostream>

int main()
{
    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian(2, 2);      //P: n*n正定矩阵,必须为稀疏矩阵SparseMatrix
    Eigen::VectorXd gradient(2);                    //Q: n*1向量
    Eigen::SparseMatrix<double> linearMatrix(2, 2); //A: m*n矩阵,必须为稀疏矩阵SparseMatrix
    Eigen::VectorXd lowerBound(2);                  //L: m*1下限向量
    Eigen::VectorXd upperBound(2);                  //U: m*1上限向量

    hessian.insert(0, 0) = 2.0; //注意稀疏矩阵的初始化方式,无法使用<<初始化
    hessian.insert(1, 1) = 2.0;
    std::cout << "hessian:" << std::endl
               << hessian << std::endl;
    gradient << -2, -2;
    //A矩阵
    linearMatrix.insert(0, 0) = 1.0; //注意稀疏矩阵的初始化方式,无法使用<<初始化
    linearMatrix.insert(1, 1) = 1.0;
    std::cout << "linearMatrix:" << std::endl
               << linearMatrix << std::endl;
    lowerBound << 1, 1;
    upperBound << 1.5, 1.5;

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(2);   //变量数n
    solver.data()->setNumberOfConstraints(2); //约束数m

    if (!solver.data()->setHessianMatrix(hessian))
        return 1;
    if (!solver.data()->setGradient(gradient))
        return 1;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
        return 1;
    if (!solver.data()->setLowerBound(lowerBound))
        return 1;
    if (!solver.data()->setUpperBound(upperBound)){
        std::cout << "i am in !!!!!!!!" << std::endl;
        return 1;
    }

    // instantiate the solver
    if (!solver.initSolver())
        return 1;

    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if (!solver.solve())
    {
        std::cout << "i am in !!!!!!!!" << std::endl;
        return 1;

    }

    QPSolution = solver.getSolution();
    std::cout << "QPSolution" << std::endl
              << QPSolution << std::endl; //输出为m*1的向量
    return 0;
}// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>
#include <iostream>

int main()
{
    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian(2, 2);      //P: n*n正定矩阵,必须为稀疏矩阵SparseMatrix
    Eigen::VectorXd gradient(2);                    //Q: n*1向量
    Eigen::SparseMatrix<double> linearMatrix(2, 2); //A: m*n矩阵,必须为稀疏矩阵SparseMatrix
    Eigen::VectorXd lowerBound(2);                  //L: m*1下限向量
    Eigen::VectorXd upperBound(2);                  //U: m*1上限向量

    hessian.insert(0, 0) = 2.0; //注意稀疏矩阵的初始化方式,无法使用<<初始化
    hessian.insert(1, 1) = 2.0;
    std::cout << "hessian:" << std::endl
               << hessian << std::endl;
    gradient << -2, -2;
    //A矩阵
    linearMatrix.insert(0, 0) = 1.0; //注意稀疏矩阵的初始化方式,无法使用<<初始化
    linearMatrix.insert(1, 1) = 1.0;
    std::cout << "linearMatrix:" << std::endl
               << linearMatrix << std::endl;
    lowerBound << 1, 1;
    upperBound << 1.5, 1.5;

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(2);   //变量数n
    solver.data()->setNumberOfConstraints(2); //约束数m

    if (!solver.data()->setHessianMatrix(hessian))
        return 1;
    if (!solver.data()->setGradient(gradient))
        return 1;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
        return 1;
    if (!solver.data()->setLowerBound(lowerBound))
        return 1;
    if (!solver.data()->setUpperBound(upperBound)){
        std::cout << "i am in !!!!!!!!" << std::endl;
        return 1;
    }

    // instantiate the solver
    if (!solver.initSolver())
        return 1;

    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if (!solver.solve())
    {
        std::cout << "i am in !!!!!!!!" << std::endl;
        return 1;

    }

    QPSolution = solver.getSolution();
    std::cout << "QPSolution" << std::endl
              << QPSolution << std::endl; //输出为m*1的向量
    return 0;
}
```

配套的Cmakelists.

```cmake
# Authors: Giulio Romualdi
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)

project(OsqpEigen-Example)

find_package(OsqpEigen)
find_package(Eigen3)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

#test
add_executable(Test src/test.cpp)
target_link_libraries(Test OsqpEigen::OsqpEigen)
```

优化问题中的warm start的意义在于采用"相关或简化问题的最优解"来作为原问题的初始值，热启动提供的初始解通常非常接近原问题的最优解或者至少可能是一个可行解，因此可更快收敛。此外，在非线性优化问题中可能存在很多局部极小值，良好的初值猜测可以有效避免陷入局部极小，warm start为优化问题提供了一个非常好的开端，例如可以采用上一次的优化结果作为下一次优化的初值。

## 函数返回值数据类型为引用

通过使用引用来替代指针，会使 C++ 程序更容易阅读和维护。C++ 函数可以返回一个引用，方式与返回一个指针类似。当函数返回一个引用时，则返回一个指向返回值的隐式指针。这样，函数就可以放在赋值语句的左边。

getReferenceStates，类似于get就是在类外访问类内的私有变量。     
