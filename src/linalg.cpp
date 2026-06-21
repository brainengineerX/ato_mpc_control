#include "ato_sim/linalg.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ato {
namespace mpc_internal {

Matrix Matrix::identity(std::size_t n) {
    Matrix m(n, n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        m(i, i) = 1.0;
    }
    return m;
}

Matrix Matrix::zeros(std::size_t rows, std::size_t cols) {
    return Matrix(rows, cols, 0.0);
}

Matrix Matrix::diag(const std::vector<double>& d) {
    Matrix m(d.size(), d.size(), 0.0);
    for (std::size_t i = 0; i < d.size(); ++i) {
        m(i, i) = d[i];
    }
    return m;
}

Matrix Matrix::operator*(const Matrix& rhs) const {
    if (cols_ != rhs.rows_) {
        throw std::runtime_error("matrix multiply: dimension mismatch");
    }
    Matrix out(rows_, rhs.cols_, 0.0);
    for (std::size_t i = 0; i < rows_; ++i) {
        for (std::size_t k = 0; k < cols_; ++k) {
            const double aik = (*this)(i, k);
            if (aik == 0.0) continue;
            for (std::size_t j = 0; j < rhs.cols_; ++j) {
                out(i, j) += aik * rhs(k, j);
            }
        }
    }
    return out;
}

Matrix Matrix::operator+(const Matrix& rhs) const {
    if (rows_ != rhs.rows_ || cols_ != rhs.cols_) {
        throw std::runtime_error("matrix add: dimension mismatch");
    }
    Matrix out(*this);
    for (std::size_t k = 0; k < data_.size(); ++k) {
        out.data_[k] += rhs.data_[k];
    }
    return out;
}

Matrix Matrix::operator-(const Matrix& rhs) const {
    if (rows_ != rhs.rows_ || cols_ != rhs.cols_) {
        throw std::runtime_error("matrix sub: dimension mismatch");
    }
    Matrix out(*this);
    for (std::size_t k = 0; k < data_.size(); ++k) {
        out.data_[k] -= rhs.data_[k];
    }
    return out;
}

Matrix Matrix::operator*(double s) const {
    Matrix out(*this);
    for (auto& v : out.data_) v *= s;
    return out;
}

Matrix Matrix::transpose() const {
    Matrix out(cols_, rows_);
    for (std::size_t i = 0; i < rows_; ++i) {
        for (std::size_t j = 0; j < cols_; ++j) {
            out(j, i) = (*this)(i, j);
        }
    }
    return out;
}

double Matrix::norm() const {
    double s = 0.0;
    for (double v : data_) s += v * v;
    return std::sqrt(s);
}

bool Matrix::solveSymmetric(const std::vector<double>& b, std::vector<double>& x) const {    if (rows_ != cols_ || b.size() != rows_) {
        return false;
    }
    const std::size_t n = rows_;
    // Bunch-Kaufman 风格：对小幅 SPD 系统用 LDL^T 分解。L 是单位下三角，D 是 1x1/2x2 块对角。
    // 简化版：直接做 LDL^T（L 单位下三角，D 对角），并加小幅正则化 ρI 防止零 D 元素。
    const double rho = 1e-12;
    std::vector<double> L(n * n, 0.0);
    std::vector<double> D(n, 0.0);

    // 复制 A 的下三角到 L 的下三角、对角到工作区
    std::vector<double> work(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        work[i] = (*this)(i, i) + rho;
    }

    bool ok = true;
    for (std::size_t j = 0; j < n; ++j) {
        for (std::size_t i = j; i < n; ++i) {
            double s = (*this)(i, j);
            for (std::size_t k = 0; k < j; ++k) {
                s -= L[i * n + k] * L[j * n + k] * D[k];
            }
            if (i == j) {
                if (std::fabs(s) < 1e-18) {
                    s = 1e-18;  // 防止 0
                }
                D[j] = s;
            } else {
                L[i * n + j] = s / D[j];
            }
        }
    }

    // 前向：L y = b
    std::vector<double> y(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        double t = b[i];
        for (std::size_t k = 0; k < i; ++k) {
            t -= L[i * n + k] * y[k];
        }
        y[i] = t;
    }
    // D z = y
    std::vector<double> z(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        z[i] = (D[i] != 0.0) ? y[i] / D[i] : 0.0;
    }
    // 后向：L^T x = z
    x.assign(n, 0.0);
    for (std::size_t i = n; i-- > 0;) {
        double t = z[i];
        for (std::size_t k = i + 1; k < n; ++k) {
            t -= L[k * n + i] * x[k];
        }
        x[i] = t;
    }
    return ok;
}

bool Matrix::solveGeneral(const std::vector<double>& b, std::vector<double>& x) const {
    if (rows_ != cols_ || b.size() != rows_) {
        return false;
    }
    const std::size_t n = rows_;
    // 增广矩阵 [A | b]，部分选主元高斯消元。
    std::vector<double> A = data_;
    std::vector<double> bb = b;
    for (std::size_t k = 0; k < n; ++k) {
        // 选主元
        std::size_t piv = k;
        double maxAbs = std::fabs(A[k * n + k]);
        for (std::size_t i = k + 1; i < n; ++i) {
            const double v = std::fabs(A[i * n + k]);
            if (v > maxAbs) { maxAbs = v; piv = i; }
        }
        if (maxAbs < 1e-14) {
            return false;  // 奇异
        }
        if (piv != k) {
            for (std::size_t j = 0; j < n; ++j) std::swap(A[k * n + j], A[piv * n + j]);
            std::swap(bb[k], bb[piv]);
        }
        // 消元
        const double akk = A[k * n + k];
        for (std::size_t i = k + 1; i < n; ++i) {
            const double f = A[i * n + k] / akk;
            if (f == 0.0) continue;
            for (std::size_t j = k; j < n; ++j) {
                A[i * n + j] -= f * A[k * n + j];
            }
            bb[i] -= f * bb[k];
        }
    }
    // 回代
    x.assign(n, 0.0);
    for (std::size_t i = n; i-- > 0;) {
        double s = bb[i];
        for (std::size_t j = i + 1; j < n; ++j) {
            s -= A[i * n + j] * x[j];
        }
        x[i] = s / A[i * n + i];
    }
    return true;
}

}  // namespace mpc_internal
}  // namespace ato
