#include "mat.h"
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>

// ==================== 构造函数和析构函数 ====================
Mat::Mat() : row(0), col(0), mat() {}

Mat::Mat(size_t Row, size_t Col) : row(Row), col(Col), mat(Row* Col, 0.0) {}

Mat::Mat(const Mat& other) : row(other.row), col(other.col), mat(other.mat) {}

Mat::Mat(Mat&& other) noexcept
    : row(other.row), col(other.col), mat(std::move(other.mat)) {
    other.row = 0;
    other.col = 0;
}

// ==================== 赋值运算符 ====================
Mat& Mat::operator=(const Mat& other) {
    if (this != &other) {
        row = other.row;
        col = other.col;
        mat = other.mat;
    }
    return *this;
}

Mat& Mat::operator=(Mat&& other) noexcept {
    if (this != &other) {
        row = other.row;
        col = other.col;
        mat = std::move(other.mat);
        other.row = 0;
        other.col = 0;
    }
    return *this;
}

// ==================== 静态工厂方法 ====================
Mat Mat::Identity(size_t size) {
    Mat result(size, size);
    for (size_t i = 0; i < size; i++) {
        result(i, i) = 1.0;
    }
    return result;
}

Mat Mat::Zero(size_t rows, size_t cols) {
    return Mat(rows, cols);
}

Mat Mat::FromVector(const std::vector<double>& data, size_t rows, size_t cols) {
    if (data.size() != rows * cols) {
        throw std::invalid_argument("Vector size does not match matrix dimensions");
    }
    Mat result(rows, cols);
    result.mat = data;
    return result;
}

// ==================== 元素访问 ====================
double& Mat::operator()(size_t i, size_t j) {
    if (i >= row || j >= col) {
        throw std::out_of_range("Matrix index out of range");
    }
    return mat[i * col + j];
}

const double& Mat::operator()(size_t i, size_t j) const {
    if (i >= row || j >= col) {
        throw std::out_of_range("Matrix index out of range");
    }
    return mat[i * col + j];
}

// ==================== 矩阵运算 ====================
Mat Mat::transpose() const {
    Mat result(col, row);
    for (size_t i = 0; i < row; i++) {
        for (size_t j = 0; j < col; j++) {
            result(j, i) = (*this)(i, j);
        }
    }
    return result;
}

double Mat::determinant() const {
    if (row != col) {
        throw std::invalid_argument("determinant() requires a square matrix");
    }

    if (row == 1) {
        return mat[0];
    }
    else if (row == 2) {
        return mat[0] * mat[3] - mat[1] * mat[2];
    }
    else if (row == 3) {
        return mat[0] * (mat[4] * mat[8] - mat[5] * mat[7]) -
            mat[1] * (mat[3] * mat[8] - mat[5] * mat[6]) +
            mat[2] * (mat[3] * mat[7] - mat[4] * mat[6]);
    }
    else {
        throw std::runtime_error("determinant() not implemented for matrices larger than 3x3");
    }
}

Mat Mat::inverse() const {
    if (row != col) {
        throw std::invalid_argument("inverse() requires a square matrix");
    }

    double det = determinant();
    if (std::abs(det) < 1e-15) {
        throw std::runtime_error("Matrix is singular, cannot compute inverse");
    }

    if (row == 1) {
        Mat result(1, 1);
        result(0, 0) = 1.0 / mat[0];
        return result;
    }
    else if (row == 2) {
        Mat result(2, 2);
        result(0, 0) = mat[3] / det;
        result(0, 1) = -mat[1] / det;
        result(1, 0) = -mat[2] / det;
        result(1, 1) = mat[0] / det;
        return result;
    }
    else if (row == 3) {
        Mat result(3, 3);
        double invDet = 1.0 / det;

        result(0, 0) = (mat[4] * mat[8] - mat[5] * mat[7]) * invDet;
        result(0, 1) = (mat[2] * mat[7] - mat[1] * mat[8]) * invDet;
        result(0, 2) = (mat[1] * mat[5] - mat[2] * mat[4]) * invDet;

        result(1, 0) = (mat[5] * mat[6] - mat[3] * mat[8]) * invDet;
        result(1, 1) = (mat[0] * mat[8] - mat[2] * mat[6]) * invDet;
        result(1, 2) = (mat[2] * mat[3] - mat[0] * mat[5]) * invDet;

        result(2, 0) = (mat[3] * mat[7] - mat[4] * mat[6]) * invDet;
        result(2, 1) = (mat[1] * mat[6] - mat[0] * mat[7]) * invDet;
        result(2, 2) = (mat[0] * mat[4] - mat[1] * mat[3]) * invDet;

        return result;
    }
    else {
        throw std::runtime_error("inverse() not implemented for matrices larger than 3x3");
    }
}

double Mat::norm() const {
    if (row != 1 && col != 1) {
        throw std::invalid_argument("norm() requires a vector");
    }

    double sum = 0.0;
    for (const auto& val : mat) {
        sum += val * val;
    }
    return std::sqrt(sum);
}

// ==================== 运算符重载 ====================
Mat& Mat::operator+=(const Mat& other) {
    if (row != other.row || col != other.col) {
        throw std::invalid_argument("Matrix dimensions do not match for addition");
    }
    for (size_t i = 0; i < mat.size(); i++) {
        mat[i] += other.mat[i];
    }
    return *this;
}

Mat& Mat::operator-=(const Mat& other) {
    if (row != other.row || col != other.col) {
        throw std::invalid_argument("Matrix dimensions do not match for subtraction");
    }
    for (size_t i = 0; i < mat.size(); i++) {
        mat[i] -= other.mat[i];
    }
    return *this;
}

Mat& Mat::operator*=(double scalar) {
    for (auto& val : mat) {
        val *= scalar;
    }
    return *this;
}

Mat& Mat::operator/=(double scalar) {
    if (std::abs(scalar) < 1e-15) {
        throw std::invalid_argument("Division by zero or near-zero scalar");
    }
    for (auto& val : mat) {
        val /= scalar;
    }
    return *this;
}

Mat Mat::operator*(double scalar) const {
    Mat result = *this;
    result *= scalar;
    return result;
}

Mat Mat::operator/(double scalar) const {
    Mat result = *this;
    result /= scalar;
    return result;
}

Mat Mat::operator+(const Mat& other) const {
    if (row != other.row || col != other.col) {
        throw std::invalid_argument("Matrix dimensions do not match for addition");
    }
    Mat result(row, col);
    for (size_t i = 0; i < mat.size(); i++) {
        result.mat[i] = mat[i] + other.mat[i];
    }
    return result;
}

Mat Mat::operator-(const Mat& other) const {
    if (row != other.row || col != other.col) {
        throw std::invalid_argument("Matrix dimensions do not match for subtraction");
    }
    Mat result(row, col);
    for (size_t i = 0; i < mat.size(); i++) {
        result.mat[i] = mat[i] - other.mat[i];
    }
    return result;
}

Mat Mat::operator*(const Mat& other) const {
    if (col != other.row) {
        throw std::invalid_argument("Matrix dimensions do not match for multiplication");
    }
    Mat result(row, other.col);
    for (size_t i = 0; i < row; i++) {
        for (size_t j = 0; j < other.col; j++) {
            double sum = 0.0;
            for (size_t k = 0; k < col; k++) {
                sum += (*this)(i, k) * other(k, j);
            }
            result(i, j) = sum;
        }
    }
    return result;
}

// ==================== 静态运算函数 ====================
Mat Mat::cross(const Mat& a, const Mat& b) {
    if (!((a.row == 3 && a.col == 1) || (a.row == 1 && a.col == 3)) ||
        !((b.row == 3 && b.col == 1) || (b.row == 1 && b.col == 3))) {
        throw std::invalid_argument("cross product requires 3D vectors");
    }

    // 提取向量分量
    double ax, ay, az, bx, by, bz;

    if (a.row == 3) {  // 列向量
        ax = a(0, 0); ay = a(1, 0); az = a(2, 0);
    }
    else {  // 行向量
        ax = a(0, 0); ay = a(0, 1); az = a(0, 2);
    }

    if (b.row == 3) {  // 列向量
        bx = b(0, 0); by = b(1, 0); bz = b(2, 0);
    }
    else {  // 行向量
        bx = b(0, 0); by = b(0, 1); bz = b(0, 2);
    }

    // 计算叉乘：a × b
    double cx = ay * bz - az * by;
    double cy = az * bx - ax * bz;
    double cz = ax * by - ay * bx;

    // 返回列向量（3x1）
    Mat result(3, 1);
    result(0, 0) = cx;
    result(1, 0) = cy;
    result(2, 0) = cz;

    return result;
}

// ==================== 友元函数 ====================
std::ostream& operator<<(std::ostream& os, const Mat& m) {
    os << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < m.row; i++) {
        os << "[ ";
        for (size_t j = 0; j < m.col; j++) {
            os << std::setw(10) << m(i, j);
            if (j < m.col - 1) os << ", ";
        }
        os << " ]" << std::endl;
    }
    return os;
}

// ==================== 非成员运算符 ====================
Mat operator*(double scalar, const Mat& m) {
    return m * scalar;
}

// 一元负号运算符
Mat Mat::operator-() const {
    Mat result(row, col);
    for (size_t i = 0; i < mat.size(); i++) {
        result.mat[i] = -mat[i];
    }
    return result;
}