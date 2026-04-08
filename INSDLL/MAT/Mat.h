#ifndef MAT_H
#define MAT_H

#include <vector>
#include <ostream>

class Mat {
private:
    size_t row;  // 行数
    size_t col;  // 列数
    std::vector<double> mat;  // 数据存储

public:
    // ==================== 构造函数和析构函数 ====================
    Mat();
    Mat(size_t Row, size_t Col);
    Mat(const Mat& other);
    Mat(Mat&& other) noexcept;
    ~Mat() = default;

    // ==================== 赋值运算符 ====================
    Mat& operator=(const Mat& other);
    Mat& operator=(Mat&& other) noexcept;

    // ==================== 静态工厂方法 ====================
    static Mat Identity(size_t size);
    static Mat Zero(size_t rows, size_t cols);
    static Mat FromVector(const std::vector<double>& data, size_t rows, size_t cols);

    // ==================== 元素访问 ====================
    double& operator()(size_t i, size_t j);
    const double& operator()(size_t i, size_t j) const;

    // ==================== 矩阵属性 ====================
    size_t rows() const { return row; }
    size_t cols() const { return col; }
    size_t size() const { return mat.size(); }
    bool empty() const { return mat.empty(); }

    // ==================== 矩阵运算 ====================
    Mat transpose() const;
    double determinant() const;
    Mat inverse() const;
    double norm() const;

    // ==================== 运算符重载（成员函数） ====================
    Mat& operator+=(const Mat& other);
    Mat& operator-=(const Mat& other);
    Mat& operator*=(double scalar);
    Mat& operator/=(double scalar);
    Mat operator*(double scalar) const;
    Mat operator/(double scalar) const;
    Mat operator+(const Mat& other) const;
    Mat operator-(const Mat& other) const;
    Mat operator*(const Mat& other) const;
    // 一元负号运算符（取负）
    Mat operator-() const;
    // ==================== 静态运算函数 ====================
    static Mat cross(const Mat& a, const Mat& b);

    // ==================== 友元函数 ====================
    friend std::ostream& operator<<(std::ostream& os, const Mat& m);
};

// ==================== 非成员运算符 ====================
Mat operator*(double scalar, const Mat& m);

#endif // MAT_H