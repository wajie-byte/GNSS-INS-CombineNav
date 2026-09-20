#ifndef MAT_H
#define MAT_H

#include <vector>
#include <ostream>

class Mat {
private:
    size_t row;
    size_t col;
    std::vector<double> mat;

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

    // ==================== 切片和子矩阵 ====================
    class SubMatrix {
    private:
        Mat& matrix;
        size_t row_start, row_end;
        size_t col_start, col_end;

    public:
        SubMatrix(Mat& mat, size_t r1, size_t r2, size_t c1, size_t c2);

        // 子矩阵赋值
        SubMatrix& operator=(const Mat& other);
        SubMatrix& operator=(const SubMatrix& other);

        // 转换为 Mat
        operator Mat() const;

        // 元素访问
        double& operator()(size_t i, size_t j);
        const double& operator()(size_t i, size_t j) const;

        // 获取子矩阵大小
        size_t rows() const { return row_end - row_start; }
        size_t cols() const { return col_end - col_start; }
    };

    class ConstSubMatrix {
    private:
        const Mat& matrix;
        size_t row_start, row_end;
        size_t col_start, col_end;

    public:
        ConstSubMatrix(const Mat& mat, size_t r1, size_t r2, size_t c1, size_t c2);

        operator Mat() const;

        double operator()(size_t i, size_t j) const;

        size_t rows() const { return row_end - row_start; }
        size_t cols() const { return col_end - col_start; }
    };

    // 切片操作（左值版本）
    SubMatrix operator()(size_t r1, size_t r2, size_t c1, size_t c2);

    // 切片操作（右值版本）
    ConstSubMatrix operator()(size_t r1, size_t r2, size_t c1, size_t c2) const;

    // 便利的 Range 类型
    struct Range {
        size_t start, end;
        Range(size_t s, size_t e) : start(s), end(e) {}
    };

    // 使用 Range 的切片操作
    SubMatrix operator()(const Range& rows, const Range& cols);
    ConstSubMatrix operator()(const Range& rows, const Range& cols) const;

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
    Mat operator-() const;

    // ==================== 静态运算函数 ====================
    static Mat cross(const Mat& a, const Mat& b);

    // ==================== 友元函数 ====================
    friend std::ostream& operator<<(std::ostream& os, const Mat& m);
};

// ==================== 非成员运算符 ====================
Mat operator*(double scalar, const Mat& m);

#endif // MAT_H