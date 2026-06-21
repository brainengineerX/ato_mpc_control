#pragma once

#include <cstddef>
#include <vector>

namespace ato {
namespace mpc_internal {

// 极简 dense 矩阵（行主序）。只支持本项目需要的运算。
// 之所以不直接用 Eigen：保持零外部依赖。
class Matrix {
public:
    Matrix() = default;
    Matrix(std::size_t rows, std::size_t cols, double value = 0.0)
        : rows_(rows), cols_(cols), data_(rows * cols, value) {}

    static Matrix identity(std::size_t n);
    static Matrix zeros(std::size_t rows, std::size_t cols);
    static Matrix diag(const std::vector<double>& d);

    std::size_t rows() const { return rows_; }
    std::size_t cols() const { return cols_; }
    bool empty() const { return data_.empty(); }

    double& operator()(std::size_t i, std::size_t j) { return data_[i * cols_ + j]; }
    double operator()(std::size_t i, std::size_t j) const { return data_[i * cols_ + j]; }

    double* data() { return data_.data(); }
    const double* data() const { return data_.data(); }

    // 矩阵乘法：C(m,k) = A(m,n) * B(n,k)。
    Matrix operator*(const Matrix& rhs) const;
    // 加减、数乘、转置、Cholesky-like 分解（小型 dense 对称正定）。
    Matrix operator+(const Matrix& rhs) const;
    Matrix operator-(const Matrix& rhs) const;
    Matrix operator*(double s) const;
    Matrix transpose() const;

    // LDL^T 分解求解 A * x = b，A 对称正定。返回 true 成功。
    bool solveSymmetric(const std::vector<double>& b, std::vector<double>& x) const;

    // 高斯消元带部分选主元求解 A * x = b（一般方阵，不必正定）。
    // 用于 KKT 系统等不定系统。返回 true 成功。
    bool solveGeneral(const std::vector<double>& b, std::vector<double>& x) const;

    // 计算 Frobenius 范数（用于残差检查）。
    double norm() const;

private:
    std::size_t rows_ = 0;
    std::size_t cols_ = 0;
    std::vector<double> data_;
};

// 向量简单包装。
using Vector = std::vector<double>;

}  // namespace mpc_internal
}  // namespace ato
