/*************************************************************************************************************
 * Matrix Class
 *  Contain the matrix class definition and operation.
 * 
 *  Notes:
 *    a. The matrix data is a 2 dimensional array, with structure:
 *      ->  0 <= i16row <= MATRIX_MAXIMUM_SIZE
 *      ->  0 <= i16col <= MATRIX_MAXIMUM_SIZE
 *      ->  floatData[MATRIX_MAXIMUM_SIZE][MATRIX_MAXIMUM_SIZE] is the memory
 *           representation of the matrix. We only use the first i16row-th
 *           and first i16col-th memory for the matrix data. The rest is unused.
 * 
 *    b. Indexing start from 0, there are 3 ways to access the data:
 *          1. A(idxRow,idxCol)       <-- The preferred way. With bounds checking.
 *          2. A[idxRow][idxCol]      <-- Slow, not recommended, but makes a cute code. With bounds checking.
 *          3. A._at(idxRow,idxCol)   <-- Just for internal function usage. Without bounds checking.
 *       
 *       Accessing index start from 0 until i16row/i16col, that is:
 *          (0 <= idxRow < i16row)     and     (0 <= idxRow < i16col).
 * 
 *    See below at "Data structure of Matrix class" at private member class definition for more information!
 * 
 * 
 * See https://github.com/pronenewbits for more!
 ************************************************************************************************************/
#ifndef MATRIX_H
#define MATRIX_H

#include <Arduino.h>

/* ASSERT is evaluated locally (without function call) to lower the computation cost */
void printErrorAndQuit(char const * str);
#define ASSERT(truth, str) { if (!(truth)) printErrorAndQuit(str); }


inline void printErrorAndQuit(char const * str)
{
    Serial.println(str);
    while(1);
}

#define EPSILON     (1e-13)
#define EPSILON_ECO (1e-8)      /* 'Economical' zero, for noisy calculation where 'somewhat zero' is good enough */




/* Define this to enable matrix bound checking */
#define MATRIX_USE_BOUNDS_CHECKING

class Matrix
{
public:
    typedef enum {
        InitMatWithZero,    /* Initialize matrix with zero data */
        NoInitMatZero
    } InitZero;
    
    /* --------------------------------------------- Basic Matrix Class functions --------------------------------------------- */
    /* Init empty matrix size _i16row x _i16col */
    Matrix(const int16_t _i16row, const int16_t _i16col, const InitZero _init = InitMatWithZero);
    /* Init matrix size _i16row x _i16col with entries initData */
    Matrix(const int16_t _i16row, const int16_t _i16col, const double* initData, const InitZero _init = InitMatWithZero);
    /* Copy constructor (for this operation --> A(B)) (copy B into A) */
    Matrix(const Matrix& old_obj);
    /* Assignment operator (for this operation --> A = B) (copy B into A) */
    Matrix& operator = (const Matrix& obj);
    /* Destructor */
    ~Matrix(void);
    /* Get internal state */
    inline int16_t getRows(void) const { return this->i16row; }
    inline int16_t getCols(void) const { return this->i16col; }
    
    
    /* ------------------------------------------- Matrix entry accessing functions ------------------------------------------- */
    /* For example: A(1,2) access the 1st row and 2nd column data of matrix A <--- The preferred way to access the matrix */
    double& operator () (const int16_t _row, const int16_t _col);
    double operator () (const int16_t _row, const int16_t _col) const;
    
    /* For example: A[1][2] access the 1st row and 2nd column data of matrix A <-- The awesome way */
    class Proxy {
        public:
            Proxy(double* _inpArr, const int16_t _maxCol) { _array.ptr = _inpArr; this->_maxCol = _maxCol; }
            Proxy(const double* _inpArr, const int16_t _maxCol) { _array.cptr = _inpArr; this->_maxCol = _maxCol; }
            double & operator [] (const int16_t _col);
            double operator [] (const int16_t _col) const;
        private:
            union { /* teehee xp */
                const double* cptr;
                double* ptr;
            } _array;
            int16_t _maxCol;
    };
    Proxy operator [] (const int16_t _row);
    const Proxy operator [] (const int16_t _row) const;
    
    
    /* ----------------------------------------- Matrix checking function declaration ----------------------------------------- */
    bool isValid(void);
    void setMatrixInvalid(void);
    bool bMatrixIsSquare();
    /* --------------------------------------------- Matrix elementary operations --------------------------------------------- */
    bool operator == (const Matrix& _compare) const;
    bool operator != (const Matrix& _compare) const;
    Matrix operator - (void) const;
    Matrix operator + (const double _scalar) const;
    Matrix operator - (const double _scalar) const;
    Matrix operator * (const double _scalar) const;
    Matrix operator / (const double _scalar) const;
    Matrix operator + (const Matrix& _matAdd) const;
    Matrix operator - (const Matrix& _matSub) const;
    Matrix operator * (const Matrix& _matMul) const;
    /* Declared outside class below */
    /* inline Matrix operator + (const double _scalar, Matrix _mat); */
    /* inline Matrix operator - (const double _scalar, Matrix _mat); */
    /* inline Matrix operator * (const double _scalar, Matrix _mat); */
    /* ----------------------------------------------- Simple Matrix operations ----------------------------------------------- */
    void roundElementToZero(const int16_t _i, const int16_t _j);
    Matrix roundElementsToZero(void);
    void setHomogen(const double _val);
    void setZero(void);
    void setRandom(const int32_t _maxRand, const int32_t _minRand);
    void setDiag(const double _val);
    void setIdentity(void);
    Matrix Transpose(void);
    bool isUnitVector(void);
    /* ------------------------------------------ Matrix/Vector insertion operations ------------------------------------------ */
    Matrix insertVector(const Matrix& _Vector, const int16_t _posCol);
    Matrix InsertSubMatrix(const Matrix& _subMatrix, const int16_t _posRow, const int16_t _posCol);
    Matrix insertSubMatrix(const Matrix& _subMatrix, const int16_t _posRow, const int16_t _posCol,
                           const int16_t _lenRow, const int16_t _lenColumn);
    Matrix insertSubMatrix(const Matrix& _subMatrix, const int16_t _posRow, const int16_t _posCol,
                           const int16_t _posRowSub, const int16_t _posColSub,
                           const int16_t _lenRow, const int16_t _lenColumn);
    /* ---------------------------------------------------- Big operations ---------------------------------------------------- */
    /* Matrix invertion using Gauss-Jordan algorithm */
    Matrix inverse(void) const;
    /* Check the definiteness of a matrix */
    bool isPositiveDefinite(const bool checkPosSemidefinite = false) const;
    /* Return the vector (Mx1 matrix) correspond with the diagonal entries of 'this' */
    Matrix getDiagonalEntries(void) const;
    /* Do the Cholesky Decomposition using Cholesky-Crout algorithm, return 'L' matrix */
    Matrix choleskyDec(void) const;
    /* Do Householder Transformation for QR Decomposition operation */
    Matrix hHouseholderTransformQR(const int16_t _rowTransform, const int16_t _colTransform);
    /* Do QR Decomposition for matrix using Householder Transformation */
    bool QRDec(Matrix& Qt, Matrix& R) const;
    /* Do back-subtitution for upper triangular matrix A & column matrix B:
     * x = BackSubtitution(&A, &B)          ; for Ax = B
     */
    Matrix backSubtitution(const Matrix& A, const Matrix& B) const;
    /* Do forward-subtitution for lower triangular matrix A & column matrix B:
     * x = ForwardSubtitution(&A, &B)       ; for Ax = B
     */
    Matrix forwardSubtitution(const Matrix& A, const Matrix& B) const;
    /* ----------------------------------------------- Matrix printing function ----------------------------------------------- */
    void print(void);
    void vPrintFull(void);
    
private:
    /* Data structure of Matrix class:
     *  0 <= i16row <= MATRIX_MAXIMUM_SIZE      ; i16row is the row of the matrix. i16row is invalid if (i16row == -1)
     *  0 <= i16col <= MATRIX_MAXIMUM_SIZE      ; i16col is the column of the matrix. i16col is invalid if (i16col == -1)
     * 
     * Accessing index start from 0 until i16row/i16col, that is:
     *  (0 <= idxRow < i16row)     and     (0 <= idxCol < i16col).
     * There are 3 ways to access the data:
     *  1. A[idxRow][idxCol]          <-- Slow, not recommended, but make a cute code. With bounds checking.
     *  2. A(idxRow, idxCol)          <-- The preferred way. With bounds checking.
     *  3. A._at(idxRow, idxCol)      <-- Just for internal function usage. Without bounds checking.
     * 
     * floatData[MATRIX_MAXIMUM_SIZE][MATRIX_MAXIMUM_SIZE] is the memory representation of the matrix. We only use the
     *  first i16row-th and first i16col-th memory for the matrix data. The rest is unused.
     * 
     * This configuration might seems wasteful (yes it is). But with this, we can make the matrix library code as cleanly
     *  as possible (like I said in the github page, I've made decision to sacrifice speed & performance to get best code
     *  readability I could get).
     * 
     * You could change the data structure of floatData if you want to make the implementation more memory efficient.
     */
    int16_t i16row;
    int16_t i16col;
    double floatData[MATRIX_MAXIMUM_SIZE][MATRIX_MAXIMUM_SIZE];
    
    /* Private way to access floatData without bound checking.
     *  TODO: For Matrix member function we could do the bound checking once at the beginning of the function, and use this
     *          to access the floatData instead of (i,j) operator. From preliminary experiment doing this only on elementary
     *          operation (experiment @2020-04-27), we can get up to 45% computation boost!!! (MPC benchmark 414 us -> 226 us)!
     */
    double& _at(const int16_t _row, const int16_t _col) { return this->floatData[_row][_col]; }
    double _at(const int16_t _row, const int16_t _col) const { return this->floatData[_row][_col]; }
};

inline Matrix operator + (const double _scalar, const Matrix& _mat);
inline Matrix operator - (const double _scalar, const Matrix& _mat);
inline Matrix operator * (const double _scalar, const Matrix& _mat);
inline Matrix MatIdentity(const int16_t _i16size);

/* ================================================= inline definition below: ================================================= */

/* ------------------------------------------ Basic Matrix Class functions ------------------------------------------ */

inline Matrix::Matrix(const int16_t _i16row, const int16_t _i16col, const InitZero _init) {
    this->i16row = _i16row;
    this->i16col = _i16col;
    
    if (_init == InitMatWithZero) {
        this->setHomogen(0.0);
    }
}

inline Matrix::Matrix(const int16_t _i16row, const int16_t _i16col, const double* initData, const InitZero _init) {
    this->i16row = _i16row;
    this->i16col = _i16col;
    
    if (_init == InitMatWithZero) {
        this->setHomogen(0.0);
    }
    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            (*this)(_i,_j) = *initData;
            initData++;
        }
    }
}

inline Matrix::Matrix(const Matrix& old_obj) {
    /* For copy contructor, we only need to copy (_i16row x _i16col) submatrix, there's no need to copy all data */
    this->i16row = old_obj.i16row;
    this->i16col = old_obj.i16col;
    
    const double *sourc = old_obj.floatData[0];
    double *desti = this->floatData[0];

    for (int16_t _i = 0; _i < i16row; _i++) {
        /* Still valid with invalid matrix ((i16row == -1) or (i16col == -1)) */
        memcpy(desti, sourc, sizeof(double)*size_t((this->i16col)));
        sourc += (MATRIX_MAXIMUM_SIZE);
        desti += (MATRIX_MAXIMUM_SIZE);
    }
}

inline Matrix& Matrix::operator = (const Matrix& obj) {
    /* For assignment operator, we only need to copy (_i16row x _i16col) submatrix, there's no need to copy all data */
    this->i16row = obj.i16row;
    this->i16col = obj.i16col;
    
    const double *sourc = obj.floatData[0];
    double *desti = this->floatData[0];

    for (int16_t _i = 0; _i < i16row; _i++) {
        /* Still valid with invalid matrix ((i16row == -1) or (i16col == -1)) */
        memcpy(desti, sourc, sizeof(double)*size_t((this->i16col)));
        sourc += (MATRIX_MAXIMUM_SIZE);
        desti += (MATRIX_MAXIMUM_SIZE);
    }
    
    return *this;
}

inline Matrix::~Matrix(void) {
    /* haven't implemented anything here, definition needed for the rule of three */
    /* (??????????`) */
}


/* ---------------------------------------- Matrix entry accessing functions ---------------------------------------- */
/* ---------------------------------------- Matrix entry accessing functions ---------------------------------------- */

/* The preferred method to access the matrix data (boring code) */
inline double& Matrix::operator () (const int16_t _row, const int16_t _col) {
    #if (defined(MATRIX_USE_BOUNDS_CHECKING))
        ASSERT((_row >= 0) && (_row < this->i16row) && (_row < MATRIX_MAXIMUM_SIZE),
               "Matrix index out-of-bounds (at row evaluation)");
        ASSERT((_col >= 0) && (_col < this->i16col) && (_col < MATRIX_MAXIMUM_SIZE),
               "Matrix index out-of-bounds (at column _column)");
    #else
        #warning("Matrix bounds checking is disabled... good luck >:3");
    #endif
    return this->floatData[_row][_col];
}
inline double Matrix::operator () (const int16_t _row, const int16_t _col) const {
    #if (defined(MATRIX_USE_BOUNDS_CHECKING))
        ASSERT((_row >= 0) && (_row < this->i16row) && (_row < MATRIX_MAXIMUM_SIZE),
               "Matrix index out-of-bounds (at row evaluation)");
        ASSERT((_col >= 0) && (_col < this->i16col) && (_col < MATRIX_MAXIMUM_SIZE),
               "Matrix index out-of-bounds (at column _column)");
    #else
        #warning("Matrix bounds checking is disabled... good luck >:3");
    #endif
    return this->floatData[_row][_col];
}

/* Ref: https://stackoverflow.com/questions/6969881/operator-overload
 * Modified to be lvalue modifiable (I know this is so dirty, but it makes the code so FABULOUS XD)
 */
inline double & Matrix::Proxy::operator [] (const int16_t _col) {
    #if (defined(MATRIX_USE_BOUNDS_CHECKING))
        ASSERT((_col >= 0) && (_col < this->_maxCol) && (_col < MATRIX_MAXIMUM_SIZE),
                "Matrix index out-of-bounds (at column evaluation)");
    #else
        #warning("Matrix bounds checking is disabled... good luck >:3");
    #endif
    return _array.ptr[_col];
}
inline double Matrix::Proxy::operator [] (const int16_t _col) const {
    #if (defined(MATRIX_USE_BOUNDS_CHECKING))
        ASSERT((_col >= 0) && (_col < this->_maxCol) && (_col < MATRIX_MAXIMUM_SIZE),
                "Matrix index out-of-bounds (at column evaluation)");
    #else
        #warning("Matrix bounds checking is disabled... good luck >:3");
    #endif
    return _array.cptr[_col];
}
inline Matrix::Proxy Matrix::operator [] (const int16_t _row) {
    #if (defined(MATRIX_USE_BOUNDS_CHECKING))
        ASSERT((_row >= 0) && (_row < this->i16row) && (_row < MATRIX_MAXIMUM_SIZE),
               "Matrix index out-of-bounds (at row evaluation)");
    #else
        #warning("Matrix bounds checking is disabled... good luck >:3");
    #endif
    return Proxy(floatData[_row], this->i16col);  /* Parsing column index for bound checking */
}
inline const Matrix::Proxy Matrix::operator [] (const int16_t _row) const {
    #if (defined(MATRIX_USE_BOUNDS_CHECKING))
        ASSERT((_row >= 0) && (_row < this->i16row) && (_row < MATRIX_MAXIMUM_SIZE),
               "Matrix index out-of-bounds (at row evaluation)");
    #else
        #warning("Matrix bounds checking is disabled... good luck >:3");
    #endif
    return Proxy(floatData[_row], this->i16col);  /* Parsing column index for bound checking */
}


/* -------------------------------------- Matrix checking function declaration -------------------------------------- */

inline bool Matrix::isValid(void) {
    /* Check whether the matrix is valid or not */
    if ((this->i16row > 0) && (this->i16row <= MATRIX_MAXIMUM_SIZE) &&
        (this->i16col > 0) && (this->i16col <= MATRIX_MAXIMUM_SIZE))
    {
        return true;
    } else {
        return false;
    }
}

inline void Matrix::setMatrixInvalid(void) {
    this->i16row = -1;
    this->i16col = -1;
}

inline bool Matrix::bMatrixIsSquare(void) {
    return (this->i16row == this->i16col);
}


/* ------------------------------------------ Matrix elementary operations ------------------------------------------ */

inline bool Matrix::operator == (const Matrix& _compare) const {
    if ((this->i16row != _compare.i16row) || (this->i16col != _compare.i16col)) {
        return false;
    }

    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            if (fabs((*this)(_i,_j) - _compare(_i,_j)) > double(EPSILON_ECO)) {
                return false;
            }
        }
    }
    return true;
}

inline bool Matrix::operator != (const Matrix& _compare) const {
    return (!(*this == _compare));
}

inline Matrix Matrix::operator - (void) const {
    Matrix _outp(this->i16row, this->i16col, NoInitMatZero);

    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            _outp(_i,_j) = -(*this)(_i,_j);
        }
    }
    return _outp;
}

inline Matrix Matrix::operator + (const double _scalar) const {
    Matrix _outp(this->i16row, this->i16col, Matrix::NoInitMatZero);

    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            _outp(_i,_j) = (*this)(_i,_j) + _scalar;
        }
    }
    return _outp;
}

inline Matrix Matrix::operator - (const double _scalar) const {
    Matrix _outp(this->i16row, this->i16col, Matrix::NoInitMatZero);

    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            _outp(_i,_j) = (*this)(_i,_j) - _scalar;
        }
    }
    return _outp;
}

inline Matrix Matrix::operator * (const double _scalar) const {
    Matrix _outp(this->i16row, this->i16col, Matrix::NoInitMatZero);

    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            _outp(_i,_j) = (*this)(_i,_j) * _scalar;
        }
    }
    return _outp;
}

inline Matrix Matrix::operator / (const double _scalar) const {
    Matrix _outp(this->i16row, this->i16col, Matrix::NoInitMatZero);

    if (fabs(_scalar) < double(EPSILON_ECO)) {
        _outp.setMatrixInvalid();
        return _outp;
    }
    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            _outp(_i,_j) = (*this)(_i,_j) / _scalar;
        }
    }
    return _outp;
}


inline Matrix operator + (const double _scalar, const Matrix& _mat) {
    Matrix _outp(_mat.getRows(), _mat.getCols(), Matrix::NoInitMatZero);

    for (int16_t _i = 0; _i < _mat.getRows(); _i++) {
        for (int16_t _j = 0; _j < _mat.getCols(); _j++) {
            _outp(_i,_j) = _scalar + _mat(_i,_j);
        }
    }
    return _outp;
}


inline Matrix operator - (const double _scalar, const Matrix& _mat) {
    Matrix _outp(_mat.getRows(), _mat.getCols(), Matrix::NoInitMatZero);

    for (int16_t _i = 0; _i < _mat.getRows(); _i++) {
        for (int16_t _j = 0; _j < _mat.getCols(); _j++) {
            _outp(_i,_j) = _scalar - _mat(_i,_j);
        }
    }
    return _outp;
}


inline Matrix operator * (const double _scalar, const Matrix& _mat) {
    Matrix _outp(_mat.getRows(), _mat.getCols(), Matrix::NoInitMatZero);

    for (int16_t _i = 0; _i < _mat.getRows(); _i++) {
        for (int16_t _j = 0; _j < _mat.getCols(); _j++) {
            _outp(_i,_j) = _scalar * _mat(_i,_j);
        }
    }
    return _outp;
}

inline Matrix Matrix::operator + (const Matrix& _matAdd) const {
    Matrix _outp(this->i16row, this->i16col, NoInitMatZero);
    if ((this->i16row != _matAdd.i16row) || (this->i16col != _matAdd.i16col)) {
        _outp.setMatrixInvalid();
        return _outp;
    }

    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            _outp(_i,_j) = (*this)(_i,_j) + _matAdd(_i,_j);
        }
    }
    return _outp;
}

inline Matrix Matrix::operator - (const Matrix& _matSub) const {
    Matrix _outp(this->i16row, this->i16col, NoInitMatZero);
    if ((this->i16row != _matSub.i16row) || (this->i16col != _matSub.i16col)) {
        _outp.setMatrixInvalid();
        return _outp;
    }

    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            _outp(_i,_j) = (*this)(_i,_j) - _matSub(_i,_j);
        }
    }
    return _outp;
}

inline Matrix Matrix::operator * (const Matrix& _matMul) const {
    Matrix _outp(this->i16row, _matMul.i16col, NoInitMatZero);
    if ((this->i16col != _matMul.i16row)) {
        _outp.setMatrixInvalid();
        return _outp;
    }

    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < _matMul.i16col; _j++) {
            _outp(_i,_j) = 0.0;
            for (int16_t _k = 0; _k < this->i16col; _k++) {
                _outp(_i,_j) += ((*this)(_i,_k) * _matMul(_k,_j));
            }
        }
    }
    return _outp;
}


inline void Matrix::roundElementToZero(const int16_t _i, const int16_t _j) {
    if (fabs((*this)(_i,_j)) < double(EPSILON_ECO)) {
        (*this)(_i,_j) = 0.0;
    }
}

inline Matrix Matrix::roundElementsToZero(void) {
    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            if (fabs((*this)(_i,_j)) < double(EPSILON_ECO)) {
                (*this)(_i,_j) = 0.0;
            }
        }
    }
    return (*this);
}

inline void Matrix::setHomogen(const double _val) {
    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            (*this)(_i,_j) = _val;
        }
    }
}

inline void Matrix::setZero(void) {
    this->setHomogen(0.0);
}

inline void Matrix::setRandom(const int32_t _maxRand, const int32_t _minRand) {
    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            (*this)(_i,_j) = double((rand() % (_maxRand - _minRand + 1)) + _minRand);
        }
    }
}

inline void Matrix::setDiag(const double _val) {
    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            if (_i == _j) {
                (*this)(_i,_j) = _val;
            } else {
                (*this)(_i,_j) = 0.0;
            }
        }
    }
}

inline void Matrix::setIdentity(void) {
    this->setDiag(1.0);
}

inline Matrix MatIdentity(const int16_t _i16size) {
    Matrix _outp(_i16size, _i16size, Matrix::NoInitMatZero);
    _outp.setDiag(1.0);   
    return _outp;
}

/* Return the transpose of the matrix */
inline Matrix Matrix::Transpose(void) {
    Matrix _outp(this->i16col, this->i16row, NoInitMatZero);
    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            _outp(_j,_i) = (*this)(_i,_j);
        }
    }
    return _outp;
}

/* Normalize the vector */
inline bool Matrix::isUnitVector(void) {
    double _normM = 0.0;
    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            _normM = _normM + ((*this)(_i,_j) * (*this)(_i,_j));
        }
    }

    /* Rounding to zero to avoid case where sqrt(0-), and _normM always positive */
    if (_normM < double(EPSILON)) {
        return false;
    }
    _normM = sqrt(_normM);
    for (int16_t _i = 0; _i < this->i16row; _i++) {
        for (int16_t _j = 0; _j < this->i16col; _j++) {
            (*this)(_i,_j) /= _normM;
        }
    }
    return true;
}


/* --------------------------------------- Matrix/Vector insertion operations --------------------------------------- */


/* Insert vector into matrix at _posCol position
 * Example: A = Matrix 3x3, B = Vector 3x1
 *
 *  C = A.InsertVector(B, 1);
 *
 *  A = [A00  A01  A02]     B = [B00]
 *      [A10  A11  A12]         [B10]
 *      [A20  A21  A22]         [B20]
 *
 *  C = [A00  B00  A02]
 *      [A10  B10  A12]
 *      [A20  B20  A22]
 */
inline Matrix Matrix::insertVector(const Matrix& _Vector, const int16_t _posCol) {
    Matrix _outp(*this);
    if ((_Vector.i16row > this->i16row) || (_Vector.i16col+_posCol > this->i16col)) {
        /* Return false */
        _outp.setMatrixInvalid();
        return _outp;
    }
    for (int16_t _i = 0; _i < _Vector.i16row; _i++) {
        _outp(_i,_posCol) = _Vector(_i,0);
    }
    return _outp;
}

/* Insert submatrix into matrix at _posRow & _posCol position
 * Example: A = Matrix 4x4, B = Matrix 2x3
 *
 *  C = A.InsertSubMatrix(B, 1, 1);
 *
 *  A = [A00  A01  A02  A03]    B = [B00  B01  B02]
 *      [A10  A11  A12  A13]        [B10  B11  B12]
 *      [A20  A21  A22  A23]
 *      [A30  A31  A32  A33]
 *
 *
 *  C = [A00  A01  A02  A03]
 *      [A10  B00  B01  B02]
 *      [A20  B10  B11  B12]
 *      [A30  A31  A32  A33]
 */
inline Matrix Matrix::InsertSubMatrix(const Matrix& _subMatrix, const int16_t _posRow, const int16_t _posCol) {
    Matrix _outp(*this);
    if (((_subMatrix.i16row+_posRow) > this->i16row) || ((_subMatrix.i16col+_posCol) > this->i16col)) {
        /* Return false */
        _outp.setMatrixInvalid();
        return _outp;
    }
    for (int16_t _i = 0; _i < _subMatrix.i16row; _i++) {
        for (int16_t _j = 0; _j < _subMatrix.i16col; _j++) {
            _outp(_i + _posRow,_j + _posCol) = _subMatrix(_i,_j);
        }
    }
    return _outp;
}

/* Insert the first _lenRow-th and first _lenColumn-th submatrix into matrix;
 *  at the matrix's _posRow and _posCol position.
 * 
 * Example: A = Matrix 4x4, B = Matrix 2x3
 *
 *  C = A.InsertSubMatrix(B, 1, 1, 2, 2);
 *
 *  A = [A00  A01  A02  A03]    B = [B00  B01  B02]
 *      [A10  A11  A12  A13]        [B10  B11  B12]
 *      [A20  A21  A22  A23]
 *      [A30  A31  A32  A33]
 *
 *
 *  C = [A00  A01  A02  A03]
 *      [A10  B00  B01  A13]
 *      [A20  B10  B11  A23]
 *      [A30  A31  A32  A33]
 */
inline Matrix Matrix::insertSubMatrix(const Matrix& _subMatrix, const int16_t _posRow, const int16_t _posCol,
                                      const int16_t _lenRow, const int16_t _lenColumn)
{
    Matrix _outp(*this);
    if (((_lenRow+_posRow) > this->i16row) || ((_lenColumn+_posCol) > this->i16col) ||
        (_lenRow > _subMatrix.i16row) || (_lenColumn > _subMatrix.i16col))
    {
        /* Return false */
        _outp.setMatrixInvalid();
        return _outp;
    }
    for (int16_t _i = 0; _i < _lenRow; _i++) {
        for (int16_t _j = 0; _j < _lenColumn; _j++) {
            _outp(_i + _posRow,_j + _posCol) = _subMatrix(_i,_j);
        }
    }
    return _outp;
}

/* Insert the _lenRow & _lenColumn submatrix, start from _posRowSub & _posColSub submatrix;
 *  into matrix at the matrix's _posRow and _posCol position.
 * 
 * Example: A = Matrix 4x4, B = Matrix 2x3
 *
 *  C = A.InsertSubMatrix(B, 1, 1, 0, 1, 1, 2);
 *
 *  A = [A00  A01  A02  A03]    B = [B00  B01  B02]
 *      [A10  A11  A12  A13]        [B10  B11  B12]
 *      [A20  A21  A22  A23]
 *      [A30  A31  A32  A33]
 *
 *  C = [A00  A01  A02  A03]
 *      [A10  B01  B02  A13]
 *      [A20  A21  A22  A23]
 *      [A30  A31  A32  A33]
 */
inline Matrix Matrix::insertSubMatrix(const Matrix& _subMatrix, const int16_t _posRow, const int16_t _posCol,
                        const int16_t _posRowSub, const int16_t _posColSub,
                        const int16_t _lenRow, const int16_t _lenColumn)
{
    Matrix _outp(*this);
    if (((_lenRow+_posRow) > this->i16row) || ((_lenColumn+_posCol) > this->i16col) ||
        ((_posRowSub+_lenRow) > _subMatrix.i16row) || ((_posColSub+_lenColumn) > _subMatrix.i16col))
    {
        /* Return false */
        _outp.setMatrixInvalid();
        return _outp;
    }
    for (int16_t _i = 0; _i < _lenRow; _i++) {
        for (int16_t _j = 0; _j < _lenColumn; _j++) {
            _outp(_i + _posRow,_j + _posCol) = _subMatrix(_posRowSub+_i,_posColSub+_j);
        }
    }
    return _outp;
}


/* ------------------------------------------------- Big operations ------------------------------------------------- */

/* Invers operation using Gauss-Jordan algorithm */
inline Matrix Matrix::inverse(void) const {
    Matrix _outp(this->i16row, this->i16col, NoInitMatZero);
    Matrix _temp(*this);
    _outp.setIdentity();
    
    /* Gauss Elimination... */
    for (int16_t _j = 0; _j < (_temp.i16row)-1; _j++) {
        for (int16_t _i = _j+1; _i < _temp.i16row; _i++) {
            if (fabs(_temp(_j,_j)) < double(EPSILON)) {
                /* Matrix is non-invertible */
                _outp.setMatrixInvalid();
                return _outp;
            }

            double _tempfloat = _temp(_i,_j) / _temp(_j,_j);

            for (int16_t _k = 0; _k < _temp.i16col; _k++) {
                _temp(_i,_k) -= (_temp(_j,_k) * _tempfloat);
                _outp(_i,_k) -= (_outp(_j,_k) * _tempfloat);

                _temp.roundElementToZero(_i, _k);
                _outp.roundElementToZero(_i, _k);
            }

        }
    }

    #if (1)
        /* Here, the _temp matrix should be an upper triangular matrix.
         * But because of rounding error, it might not.
         */
        for (int16_t _i = 1; _i < _temp.i16row; _i++) {
            for (int16_t _j = 0; _j < _i; _j++) {
                _temp(_i,_j) = 0.0;
            }
        }
    #endif


    /* Jordan... */
    for (int16_t _j = (_temp.i16row)-1; _j > 0; _j--) {
        for (int16_t _i = _j-1; _i >= 0; _i--) {
            if (fabs(_temp(_j,_j)) < double(EPSILON)) {
                /* Matrix is non-invertible */
                _outp.setMatrixInvalid();
                return _outp;
            }

            double _tempfloat = _temp(_i,_j) / _temp(_j,_j);
            _temp(_i,_j) -= (_temp(_j,_j) * _tempfloat);
            _temp.roundElementToZero(_i, _j);

            for (int16_t _k = (_temp.i16row - 1); _k >= 0; _k--) {
                _outp(_i,_k) -= (_outp(_j,_k) * _tempfloat);
                _outp.roundElementToZero(_i, _k);
            }
        }
    }


    /* Normalization */
    for (int16_t _i = 0; _i < _temp.i16row; _i++) {
        if (fabs(_temp(_i,_i)) < double(EPSILON)) {
            /* Matrix is non-invertible */
            _outp.setMatrixInvalid();
            return _outp;
        }

        double _tempfloat = _temp(_i,_i);
        _temp(_i,_i) = 1.0;

        for (int16_t _j = 0; _j < _temp.i16row; _j++) {
            _outp(_i,_j) /= _tempfloat;
        }
    }
    return _outp;
}

/* Use elemetary row operation to reduce the matrix into upper triangular form
 *  (like in the first phase of gauss-jordan algorithm).
 * 
 * Useful if we want to check whether the matrix is positive definite or not
 *  (useful before calling CholeskyDec function).
 */
inline bool Matrix::isPositiveDefinite(const bool checkPosSemidefinite) const {
    bool _posDef, _posSemiDef;
    Matrix _temp(*this);
    
    /* Gauss Elimination... */
    for (int16_t _j = 0; _j < (_temp.i16row)-1; _j++) {
        for (int16_t _i = _j+1; _i < _temp.i16row; _i++) {
            if (fabs(_temp(_j,_j)) < double(EPSILON)) {
                /* Q: Do we still need to check this?
                 * A: idk, it's 3 AM. I need sleep :<
                 * 
                 * NOTE TO FUTURE SELF: Confirm it!
                 */
                return false;
            }
            
            double _tempfloat = _temp(_i,_j) / _temp(_j,_j);
            
            for (int16_t _k = 0; _k < _temp.i16col; _k++) {
                _temp(_i,_k) -= (_temp(_j,_k) * _tempfloat);
                _temp.roundElementToZero(_i, _k);
            }

        }
    }
    
    _posDef = true;
    _posSemiDef = true;
    for (int16_t _i = 0; _i < _temp.i16row; _i++) {
        if (_temp(_i,_i) < double(EPSILON)) {
            /* false if less than 0+ (zero included) */
            _posDef = false;
        }
        if (_temp(_i,_i) < -double(EPSILON)) {
            /* false if less than 0- (zero is not included) */
            _posSemiDef = false;
        }
    }
    
    if (checkPosSemidefinite) {
        return _posSemiDef;
    } else {
        return _posDef;
    }
}

/* For square matrix 'this' with size MxM, return vector Mx1 with entries
 * correspond with diagonal entries of 'this'.
 * 
 *   Example:    this = [a11 a12 a13]
 *                      [a21 a22 a23]
 *                      [a31 a32 a33]
 * 
 * out = this.GetDiagonalEntries() = [a11]
 *                                   [a22]
 *                                   [a33]
 */
inline Matrix Matrix::getDiagonalEntries(void) const {
    Matrix _temp(this->i16row, 1, NoInitMatZero);
    
    if (this->i16row != this->i16col) {
        _temp.setMatrixInvalid();
        return _temp;
    }
    for (int16_t _i = 0; _i < this->i16row; _i++) {
        _temp(_i,0) = (*this)(_i,_i);
    }
    return _temp;
}

/* Do the Cholesky Decomposition using Cholesky-Crout algorithm.
 * 
 *      A = L*L'     ; A = real, positive definite, and symmetry MxM matrix
 *
 *      L = A.CholeskyDec();
 *
 *      CATATAN! NOTE! The symmetry property is not checked at the beginning to lower
 *          the computation cost. The processing is being done on the lower triangular
 *          component of _A. Then it is assumed the upper triangular is inherently
 *          equal to the lower end.
 *          (as a side note, Scilab & MATLAB is using Lapack routines DPOTRF that process
 *           the upper triangular of _A. The result should be equal mathematically if A
 *           is symmetry).
 */
inline Matrix Matrix::choleskyDec(void) const {
    double _tempFloat;

    /* Note that _outp need to be initialized as zero matrix */
    Matrix _outp(this->i16row, this->i16col, InitMatWithZero);
    
    if (this->i16row != this->i16col) {
        _outp.setMatrixInvalid();
        return _outp;
    }
    for (int16_t _j = 0; _j < this->i16col; _j++) {
        for (int16_t _i = _j; _i < this->i16row; _i++) {
            _tempFloat = (*this)(_i,_j);
            if (_i == _j) {
                for (int16_t _k = 0; _k < _j; _k++) {
                    _tempFloat = _tempFloat - (_outp(_i,_k) * _outp(_i,_k));
                }
                if (_tempFloat < -double(EPSILON)) {
                    /* Matrix is not positif (semi)definit */
                    _outp.setMatrixInvalid();
                    return _outp;
                }
                /* Rounding to zero to avoid case where sqrt(0-) */
                if (fabs(_tempFloat) < double(EPSILON)) {
                    _tempFloat = 0.0;
                }
                _outp(_i,_i) = sqrt(_tempFloat);
            } else {
                for (int16_t _k = 0; _k < _j; _k++) {
                    _tempFloat = _tempFloat - (_outp(_i,_k) * _outp(_j,_k));
                }
                if (fabs(_outp(_j,_j)) < double(EPSILON)) {
                    /* Matrix is not positif definit */
                    _outp.setMatrixInvalid();
                    return _outp;
                }
                _outp(_i,_j) = _tempFloat / _outp(_j,_j);
            }
        }
    }
    return _outp;
}

/* Do the Householder Transformation for QR Decomposition operation.
 *              out = HouseholderTransformQR(A, i, j)
 */
inline Matrix Matrix::hHouseholderTransformQR(const int16_t _rowTransform, const int16_t _colTransform) {
    double _tempFloat;
    double _xLen;
    double _x1;
    double _u1;
    double _vLen2;
    
    /* Note that _outp & _vectTemp need to be initialized as zero matrix */
    Matrix _outp(this->i16row, this->i16row, InitMatWithZero);
    Matrix _vectTemp(this->i16row, 1, InitMatWithZero);
    
    if ((_rowTransform >= this->i16row) || (_colTransform >= this->i16col)) {
        _outp.setMatrixInvalid();
        return _outp;
    }
    
    /* Until here:
     *
     * _xLen    = ||x||            = sqrt(x1^2 + x2^2 + .. + xn^2)
     * _vLen2   = ||u||^2 - [u1^2] = x2^2 + .. + xn^2
     * _vectTemp= [0 0 0 .. x1=0 x2 x3 .. xn]'
     */
    _x1 = (*this)(_rowTransform,_colTransform);
    _xLen = _x1*_x1;
    _vLen2 = 0.0;
    for (int16_t _i = _rowTransform+1; _i < this->i16row; _i++) {
        _vectTemp(_i,0) = (*this)(_i,_colTransform);

        _tempFloat = _vectTemp(_i,0) * _vectTemp(_i,0);
        _xLen  += _tempFloat;
        _vLen2 += _tempFloat;
    }
    _xLen = sqrt(_xLen);
    
    /* u1    = x1+(-sign(x1))*xLen */
    if (_x1 < 0.0) {
        _u1 = _x1+_xLen;
    } else {
        _u1 = _x1-_xLen;
    }
    
    /* Solve vlen2 & tempHH */
    _vLen2 += (_u1*_u1);
    _vectTemp(_rowTransform,0) = _u1;
    
    if (fabs(_vLen2) < double(EPSILON_ECO)) {
        /* x vector is collinear with basis vector e, return result = I */
        _outp.setIdentity();
    } else {
        /* P = -2*(u1*u1')/v_len2 + I */
        /* PR TODO: We need to investigate more on this */
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            _tempFloat = _vectTemp(_i,0);
            if (fabs(_tempFloat) > double(EPSILON)) {
                for (int16_t _j = 0; _j < this->i16row; _j++) {
                    if (fabs(_vectTemp(_j,0)) > double(EPSILON)) {
                        _outp(_i,_j) = _vectTemp(_j,0);
                        _outp(_i,_j) = _outp(_i,_j) * _tempFloat;
                        _outp(_i,_j) = _outp(_i,_j) * (-2.0/_vLen2);
                    }
                }
            }
            _outp(_i,_i) = _outp(_i,_i) + 1.0;
        }
    }
    return _outp;
}

/* Do the QR Decomposition for matrix using Householder Transformation.
 *                      A = Q*R
 * 
 * PERHATIAN! CAUTION! The matrix calculated by this function are Q' and R (Q transpose and R).
 *  Because QR Decomposition usually used to calculate solution for least-squares equation
 *  (that need Q'), we don't do the transpose of Q inside this routine to lower the
 *  computation cost (user need to transpose outside if they want Q).
 * 
 * Example of using QRDec to solve least-squares:
 *                      Ax = b
 *                   (QR)x = b
 *                      Rx = Q'b    --> Afterward use back-subtitution to solve x
 */
inline bool Matrix::QRDec(Matrix& Qt, Matrix& R) const {
    Matrix Qn(Qt.i16row, Qt.i16col, NoInitMatZero);
    if ((this->i16row < this->i16col) || (!Qt.bMatrixIsSquare()) || (Qt.i16row != this->i16row) ||
        (R.i16row != this->i16row) || (R.i16col != this->i16col))
    {
        Qt.setMatrixInvalid();
        R.setMatrixInvalid();
        return false;
    }
    R = (*this);
    Qt.setIdentity();
    for (int16_t _i = 0; (_i < (this->i16row - 1)) && (_i < this->i16col-1); _i++) {
        Qn  = R.hHouseholderTransformQR(_i, _i);
        if (!Qn.isValid()) {
            Qt.setMatrixInvalid();
            R.setMatrixInvalid();
            return false;
        }
        Qt = Qn * Qt;
        R  = Qn * R;
    }
    Qt.roundElementsToZero();
    for (int16_t _i = 1; ((_i < R.i16row) && (_i < R.i16col)); _i++) {
        for (int16_t _j = 0; _j < _i; _j++) {
            R(_i, _j) = 0.0;
        }
    }
    
    return true;
}

/* Do the back-subtitution operation for upper triangular matrix A & column matrix B to solve x:
 *                      Ax = B
 * 
 * x = BackSubtitution(&A, &B);
 *
 * CATATAN! NOTE! To lower the computation cost, we don't check that A is a upper triangular
 *  matrix (it's assumed that user already make sure of that before calling this routine).
 */
inline Matrix Matrix::backSubtitution(const Matrix& A, const Matrix& B) const {
    Matrix _outp(A.i16row, 1, NoInitMatZero);
    if ((A.i16row != A.i16col) || (A.i16row != B.i16row)) {
        _outp.setMatrixInvalid();
        return _outp;
    }

    for (int16_t _i = A.i16col-1; _i >= 0; _i--) {
        _outp(_i,0) = B(_i,0);
        for (int16_t _j = _i + 1; _j < A.i16col; _j++) {
            _outp(_i,0) = _outp(_i,0) - A(_i,_j)*_outp(_j,0);
        }
        if (fabs(A(_i,_i)) < double(EPSILON)) {
            _outp.setMatrixInvalid();
            return _outp;
        }
        _outp(_i,0) = _outp(_i,0) / A(_i,_i);
    }

    return _outp;
}

/* Do the forward-subtitution operation for lower triangular matrix A & column matrix B to solve x:
 *                      Ax = B
 * 
 * x = ForwardSubtitution(&A, &B);
 *
 * CATATAN! NOTE! To lower the computation cost, we don't check that A is a lower triangular
 *  matrix (it's assumed that user already make sure of that before calling this routine).
 */
inline Matrix Matrix::forwardSubtitution(const Matrix& A, const Matrix& B) const {
    Matrix _outp(A.i16row, 1, NoInitMatZero);
    if ((A.i16row != A.i16col) || (A.i16row != B.i16row)) {
        _outp.setMatrixInvalid();
        return _outp;
    }

    for (int16_t _i = 0; _i < A.i16row; _i++) {
        _outp(_i,0) = B(_i,0);
        for (int16_t _j = 0; _j < _i; _j++) {
            _outp(_i,0) = _outp(_i,0) - A(_i,_j)*_outp(_j,0);
        }
        if (fabs(A(_i,_i)) < double(EPSILON)) {
            _outp.setMatrixInvalid();
            return _outp;
        }
        _outp(_i,0) = _outp(_i,0) / A(_i,_i);
    }
    return _outp;
}


/* -------------------------------------------- Matrix printing function -------------------------------------------- */
    inline void Matrix::print(void) {
        char _bufSer[10];
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            Serial.print("[ ");
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                snprintf(_bufSer, sizeof(_bufSer)-1, "%2.4f ", (*this)(_i,_j));
                Serial.print(_bufSer);
            }
            Serial.println("]");
        }
        Serial.println("");
    }
    inline void Matrix::vPrintFull(void) {
        char _bufSer[32];
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            Serial.print("[ ");
            for (int16_t _j = 0; _j < this->i16col; _j++) {
                snprintf(_bufSer, sizeof(_bufSer)-1, "%e ", (*this)(_i,_j));
                Serial.print(_bufSer);
            }
            Serial.println("]");
        }
        Serial.println("");
    }
    
    
#endif // MATRIX_H