/*
 *  CNC: Concurrent Number Cruncher
 *  Copyright (C) 2008 GOCAD/ASGA, INRIA/ALICE
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  If you modify this software, you should include a notice giving the
 *  name of the person performing the modification, the date of modification,
 *  and the reason for such modification.
 *
 *  Contact: Luc Buatois
 *
 *     buatois@gocad.org
 *
 *     ASGA-INPL Bt. G
 *     Rue du Doyen Marcel Roubault - BP 40
 *     54501 VANDOEUVRE LES NANCY
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs.
 */

#ifndef CNC_SPARSE_MATRIX_H
#define CNC_SPARSE_MATRIX_H

#include "cnc_utils.h"

#include <set>
#include <vector>
#include <algorithm>
#include <assert.h>

#define CNC_ROW_COLUMN_RESERVED	32

enum { CNC_alignment_for_SSE2 = 16} ;

// -------------------------------------------------------------------------- //
// Dynamic Sparse matrix data structures									  //
// -------------------------------------------------------------------------- //

/**
 * A coefficient of a SparseMatrix. (currently FLOAT data ONLY)
 */
template <typename T>
class CNCCoeff
{
public:
    T a ;
    long index ;
} ;

//---------------------------------------------------------------------------//
template <typename T> class CNCSparseMatrix;
/**
 * A row or a column of a SparseMatrix. SparseRowColumn is
 * compressed, and stored in the form of a list of
 * (value,index) couples.
 */
template <typename T>
class CNCSparseRowColumn
{
public:
    friend class CNCSparseMatrix<T>;

    CNCSparseRowColumn()
    {
        coeff_ = CNCallocate<CNCCoeff<T> >(CNC_ROW_COLUMN_RESERVED) ;
        nb_coeffs_ = 0 ;
        capacity_ = CNC_ROW_COLUMN_RESERVED ;
    }

    ~CNCSparseRowColumn()
    {
        CNCdeallocate<CNCCoeff<T> >(coeff_) ;
    }

    long nb_coeffs() const
    {
        return nb_coeffs_ ;
    }

    CNCCoeff<T>& coeff(long ii)
    {
        return coeff_[ii] ;
    }

    const CNCCoeff<T>& coeff(long ii) const
    {
        return coeff_[ii] ;
    }

    long capacity() const
    {
        return capacity_;
    }

    /** a_{index} <- a_{index} + val */
    void add(long index, T val) ;

    /** sorts the coefficients by increasing index */
    void sort() ;


    /**
     * removes all the coefficients and frees the allocated
     * space.
     */
    void clear();

    /**
     * removes all the coefficients, but keeps the
     * allocated space, that will be used by subsequent
     * calls to add().
     */
    void zero();

    /**
     * reserve specific size of space for coefficients
     */
    void reserve(int size);

    /**
    *	push coefficients at the back
    */
    void push_coeff(long index, T val);
    void push_coeff_unsafe(const CNCCoeff<T> &cf);
    void push_coeff_unsafe(long index, T val);

protected:
    void grow() ;

private:
    CNCCoeff<T>* coeff_ ;
    long nb_coeffs_ ;
    long capacity_ ;
} ;

//---------------------------------------------------------------------------//
template <typename T>
class CNCSparseMatrix
{
public:

    enum Storage { NONE, ROWS, COLUMNS, ROWS_AND_COLUMNS} ;

    // constructors / destructor

    /**
     * Constructs a m*n sparse matrix.
     * @param Storage can be one of ROWS, COLUMNS, ROWS_AND_COLUMNS
     */
    CNCSparseMatrix(long m, long n, Storage storage = ROWS) ;

    /**
     * Constructs a n*n sparse matrix, row storage is used,
     * Non symmetric storage is used
     */
    CNCSparseMatrix(long n) ;

    /**
     * Constructs a square n*n sparse matrix.
     * @param Storage can be one of ROWS, COLUMNS, ROWS_AND_COLUMNS
     * @param symmetric_storage if set, only entries a_ij such
     *   that j <= i are stored.
     */
    CNCSparseMatrix(long n, Storage storage, bool symmetric_storage) ;

    CNCSparseMatrix() ;

    ~CNCSparseMatrix() ;

    // access

    long m() const
    {
        return m_ ;
    }

    long n() const
    {
        return n_ ;
    }

    /** number of non-zero coefficients */
    long nnz() const ;

    bool rows_are_stored() const
    {
        return rows_are_stored_ ;
    }

    bool columns_are_stored() const
    {
        return columns_are_stored_ ;
    }

    Storage storage() const
    {
        return storage_ ;
    }

    bool has_symmetric_storage() const
    {
        return symmetric_storage_ ;
    }

    bool is_square() const
    {
        return (m_ == n_) ;
    }

    bool is_symmetric() const
    {
        return (symmetric_storage_ || symmetric_tag_) ;
    }

    /**
     * For symmetric matrices that are not stored in symmetric mode,
     * one may want to give a hint that the matrix is symmetric.
     */
    void set_symmetric_tag(bool x)
    {
        symmetric_tag_ = x ;
    }

    /**
     * storage should be one of ROWS, ROWS_AND_COLUMNS
     * @param i index of the row, in the range [0, m-1]
     */
    CNCSparseRowColumn<T>& row(long i)
    {
        return row_[i] ;
    }

    /**
     * storage should be one of ROWS, ROWS_AND_COLUMNS
     * @param i index of the row, in the range [0, m-1]
     */
    const CNCSparseRowColumn<T>& row(long i) const
    {
        return row_[i] ;
    }

    /**
     * storage should be one of COLUMN, ROWS_AND_COLUMNS
     * @param i index of the column, in the range [0, n-1]
     */
    CNCSparseRowColumn<T>& column(long j)
    {
        return column_[j] ;
    }

    /**
     * storage should be one of COLUMNS, ROWS_AND_COLUMNS
     * @param i index of the column, in the range [0, n-1]
     */
    const CNCSparseRowColumn<T>& column(long j) const
    {
        return column_[j] ;
    }

    /**
     * aij <- aij + val
     */
    void add(long i, long j, T val)
    {
        if(symmetric_storage_ && j > i)
        {
            return ;
        }

        /*if(i == j) {
            diag_[i] += val ;
        } */
        if(rows_are_stored_)
        {
            row(i).add(j, val) ;
        }

        if(columns_are_stored_)
        {
            column(j).add(i, val) ;
        }
    }

    /** sorts rows and columns by increasing coefficients */
    void sort() ;


    /**
     * removes all the coefficients and frees the allocated
     * space.
     */
    void clear() ;

    /**
     * removes all the coefficients, but keeps the allocated
     * storage, that will be used by subsequent calls to add().
     */
    void zero() ;

    void allocate(
        long m, long n, Storage storage, bool symmetric = false
    ) ;

    void deallocate() ;

    /**
    *  pre-multiply the transpose matrix of itself and add to the
    *  destination matrix
    */
    void premult_transpose_add(CNCSparseMatrix &mat) const;

    /**
    *  transpose and post-multiply the vector
    */
    void transpose_postmult(T *b, T *v) const;

    /**
    *  post-multiply the vector
    */
    void postmult(T *b, T *v) const;

    /**
     *	v += A * b
     */
    void postmult_add(T *b, T *v) const;

    /**
    *	push coefficients the back of row, used for pushing elements
    *	row-by-row	-add by Hao~ hao.deng@ymail.com
    */
    void push_coeff_row(long i, long j, T val)
    {
        row_[i].push_coeff_unsafe(j, val);
    }

    /**
    *	make a copy	from parameter matrix
    *	-add by Hao~ hao.deng@ymail.com
    */
    CNCSparseMatrix<T> & copy(const CNCSparseMatrix<T> & mat);

    /**
    * A(i, j) := A(i, j) + M(i, j)
    */
    CNCSparseMatrix<T> & operator += (const CNCSparseMatrix<T> & mat);
    /**
    * A(i, j) := A(i, j) - M(i, j)
    */
    CNCSparseMatrix<T> & operator -= (const CNCSparseMatrix<T> & mat);

    /**
    * A(i, j) := A(i, j) * s
    */
    CNCSparseMatrix<T> & operator *= (const T &s);


protected:
    long m_ ;
    long n_ ;

    CNCSparseRowColumn<T>* row_ ;
    CNCSparseRowColumn<T>* column_ ;

    Storage storage_ ;
    bool rows_are_stored_ ;
    bool columns_are_stored_ ;
    bool symmetric_storage_ ;
    bool symmetric_tag_ ;

    // SparseMatrix cannot be copied.
    CNCSparseMatrix(const CNCSparseMatrix<T>& rhs) ;
    CNCSparseMatrix& operator=(const CNCSparseMatrix<T>& rhs) ;
} ;

#if DO_NOT_NEEDED
//---------------------------------------------------------------------------//

template <class T> inline void convert_matrix(
    const CNCSparseMatrix& rhs, CNCSparseMatrixCRS<T>& A, bool separate_diag = true) ;

template <class T, int BM, int BN> inline void convert_matrix(
    const CNCSparseMatrix& rhs, CNCSparseMatrixBCRS<T, BM, BN>& A) ;


//---------------------------------------------------------------------------//

template <class T> inline void convert_matrix(
    const CNCSparseMatrix& rhs, CNCSparseMatrixCRS<T>& A, bool separate_diag)
{

    A.separate_diag = separate_diag ;
    A.symmetric_storage = rhs.has_symmetric_storage() ;
    A.N = rhs.n() ;
    A.rowptr.allocate(rhs.m() + 1) ;
    unsigned int nnz = rhs.nnz() ;

    if(separate_diag)
    {
        nnz -= rhs.diag_size() ;
    }

    A.colind.allocate(nnz) ;
    A.a.allocate(nnz, CNC_alignment_for_SSE2) ;
    A.diag.allocate(rhs.diag_size(), CNC_alignment_for_SSE2) ;

    unsigned int cur = 0 ;

    for(int i = 0; i < rhs.m(); i++)
    {
        A.rowptr[i] = cur ;
        const CNCSparseRowColumn & R = rhs.row(i) ;

        for(int jj = 0; jj < R.nb_coeffs(); jj++)
        {
            if(!separate_diag || (R.coeff(jj).index != i))
            {
                A.a[cur] = T(R.coeff(jj).a) ;
                A.colind[cur] = R.coeff(jj).index ;
                cur++ ;
            }
        }
    }

    A.rowptr[rhs.m()] = nnz ;

    for(int i = 0; i < rhs.diag_size(); i++)
    {
        A.diag[i] = T(rhs.diag(i)) ;
    }
}


//---------------------------------------------------------------------------//

template <class T, int BM, int BN> inline void convert_matrix(
    const CNCSparseMatrix& rhs, CNCSparseMatrixBCRS<T, BM, BN>& A)
{

    unsigned int BLOC_SIZE = CNCSparseMatrixBCRS<T, BM, BN>::BLOC_SIZE ;

    // Compute number of bloc rows and bloc columns
    unsigned int M = rhs.m() / BM ;

    if((rhs.m() % BM) != 0)
    {
        M++ ;
    }

    unsigned int N = rhs.n() / BN ;

    if((rhs.n() % BN) != 0)
    {
        N++ ;
    }

    A.N_ = N ;

    // Step 1: determine blocs to use
    CNCArray1d< std::set<unsigned int> > row_blocs(M) ;

    for(int i = 0; i < rhs.m(); i++)
    {
        unsigned int I = i / BM ;
        const CNCSparseRowColumn & Ri = rhs.row(i) ;

        for(int jj = 0 ; jj < Ri.nb_coeffs(); jj++)
        {
            unsigned int j = Ri.coeff(jj).index ;
            unsigned int J = j / BN ;
            row_blocs[I].insert(J) ;
        }
    }

    // Step 2: initialize rowptr
    A.rowptr.allocate(M + 1) ;
    A.rowptr[0] = 0 ;

    for(unsigned int I = 0; I < M; I++)
    {
        A.rowptr[I + 1] = (long)(A.rowptr[I] + row_blocs[I].size()) ;
    }

    unsigned int NNZ = A.rowptr[M] ;

    // Step 3: initialize colind
    A.colind.allocate(NNZ) ;
    unsigned int cur = 0 ;

    for(unsigned int I = 0; I < M; I++)
    {
        for(std::set<unsigned int>::iterator it = row_blocs[I].begin(); it != row_blocs[I].end(); it++)
        {
            A.colind[cur++] = (*it) ;
        }
    }

    // Step 4: initialize a
    A.a.allocate(NNZ * BLOC_SIZE, CNC_alignment_for_SSE2) ;
    A.a.set_all(0.0) ;

    for(int i = 0; i < rhs.m(); i++)
    {
        unsigned int I = i / BM ;
        unsigned int di = i % BM ;
        const CNCSparseRowColumn & Ri = rhs.row(i) ;

        for(int jj = 0; jj < Ri.nb_coeffs(); jj++)
        {
            unsigned int j = Ri.coeff(jj).index ;
            unsigned int J = j / BN ;
            unsigned int dj = j % BN ;
            bool found = false ;

            for(int K = A.rowptr[I]; K < A.rowptr[I + 1]; K++)
            {
                if(A.colind[K] == J)
                {
                    A.a[K * BLOC_SIZE + di * BN + dj] = Ri.coeff(jj).a ;
                    found = true ;
                    break ;
                }
            }
        }
    }

    // Step 5: initialize diag
    A.diag.allocate(rhs.diag_size()) ;

    for(unsigned int i = 0; i < A.diag.size(); i++)
    {
        A.diag[i] = rhs.diag(i) ;
    }
}

#endif	// DO_NOT_NEEDED
//---------------------------------------------------------------------------//

// -------------------------------------------------------------------------- //
// Dynamic Sparse matrix data structures									  //
// -------------------------------------------------------------------------- //
template <typename T>
void CNCSparseRowColumn<T>::add(long index, T val)
{
    CNCCoeff<T>* coeff = NULL ;

    // Search for a_{index}
    for(long ii = 0; ii < nb_coeffs_; ii++)
    {
        if(coeff_[ii].index == index)
        {
            coeff = &(coeff_[ii]) ;
            break ;
        }
    }

    if(coeff != NULL)
    {
        coeff->a += val ;
    }
    else
    {
        nb_coeffs_++ ;

        if(nb_coeffs_ > capacity_)
        {
            grow() ;
        }

        coeff = &(coeff_[nb_coeffs_ - 1]) ;
        coeff->a     = val ;
        coeff->index = index ;
    }
}

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseRowColumn<T>::grow()
{
    long old_capacity = capacity_ ;
    capacity_ = capacity_ * 2 ;
    CNCreallocate<CNCCoeff<T> >(
        coeff_, old_capacity, capacity_
    ) ;
}

//---------------------------------------------------------------------------//
template <typename T>
class CNCCoeffIndexCompare
{
public:
    bool operator()(const CNCCoeff<T>& c1, const CNCCoeff<T>& c2)
    {
        return c1.index < c2.index ;
    }
} ;

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseRowColumn<T>::sort()
{
    CNCCoeff<T>* begin = coeff_ ;
    CNCCoeff<T>* end   = coeff_ + nb_coeffs_ ;
    std::sort(begin, end, CNCCoeffIndexCompare<T>()) ;
}

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseRowColumn<T>::clear()
{
    CNCdeallocate<CNCCoeff<T> >(coeff_) ;
    coeff_ = CNCallocate<CNCCoeff<T> >(2) ;
    nb_coeffs_ = 0 ;
    capacity_ = 2 ;
}

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseRowColumn<T>::zero()
{
    memset(coeff_, 0, sizeof(CNCCoeff<T>) * nb_coeffs_);
    nb_coeffs_ = 0 ;
}

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseRowColumn<T>::reserve(int size)
{
    CNCdeallocate<CNCCoeff<T> >(coeff_) ;
    coeff_ = CNCallocate<CNCCoeff<T> >(size) ;
    nb_coeffs_ = 0 ;
    capacity_ = size ;
}

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseRowColumn<T>::push_coeff_unsafe(long index, T val)
{
    // the following operation is dangerous for the sake of efficiency
    CNCCoeff<T> *pCoeff = coeff_ + nb_coeffs_++;
    pCoeff->a = val;
    pCoeff->index = index;
}

template <typename T>
void CNCSparseRowColumn<T>::push_coeff_unsafe(const CNCCoeff<T> &cf)
{
    // the following operation is dangerous for the sake of efficiency
    CNCCoeff<T> *pCoeff = coeff_ + nb_coeffs_++;
    pCoeff->a     = cf.a ;
    pCoeff->index = cf.index ;
}

template <typename T>
void CNCSparseRowColumn<T>::push_coeff(long index, T val)
{
    CNCCoeff<T>* pCoeff = NULL ;
    nb_coeffs_++ ;

    if(nb_coeffs_ > capacity_)
        grow() ;

    pCoeff = &(coeff_[nb_coeffs_ - 1]) ;
    pCoeff->a     = val ;
    pCoeff->index = index ;
}

//---------------------------------------------------------------------------//
template <typename T>
CNCSparseMatrix<T>::CNCSparseMatrix(long m, long n, Storage storage)
{
    storage_ = NONE ;
    allocate(m, n, storage, false) ;
}

//---------------------------------------------------------------------------//
template <typename T>
CNCSparseMatrix<T>::CNCSparseMatrix(
    long n, Storage storage, bool symmetric_storage
)
{
    storage_ = NONE ;
    allocate(n, n, storage, symmetric_storage) ;
}

//---------------------------------------------------------------------------//
template <typename T>
CNCSparseMatrix<T>::CNCSparseMatrix(long n)
{
    m_ = 0 ;
    n_ = 0 ;

    row_ = NULL ;
    column_ = NULL ;

    storage_ = ROWS ;
    allocate(n, n, storage_, false) ;
}

//---------------------------------------------------------------------------//
template <typename T>
CNCSparseMatrix<T>::~CNCSparseMatrix()
{
    deallocate() ;
}

//---------------------------------------------------------------------------//
template <typename T>
CNCSparseMatrix<T>::CNCSparseMatrix()
{
    m_ = 0 ;
    n_ = 0 ;

    row_ = NULL ;
    column_ = NULL ;

    storage_ = NONE ;
    rows_are_stored_ = false ;
    columns_are_stored_ = false ;
    symmetric_storage_ = false ;
    symmetric_tag_ = false ;
}

//---------------------------------------------------------------------------//
template <typename T>
long CNCSparseMatrix<T>::nnz() const
{
    long result = 0 ;

    if(rows_are_stored())
    {
        for(long i = 0; i < m(); i++)
        {
            result += row(i).nb_coeffs() ;
        }
    }
    else if(columns_are_stored())
    {
        for(long j = 0; j < n(); j++)
        {
            result += column(j).nb_coeffs() ;
        }
    }
    else
    {
    }

    return result ;
}

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseMatrix<T>::deallocate()
{
    m_ = 0 ;
    n_ = 0 ;

    if(row_ != NULL) delete[] row_ ;

    if(column_ != NULL) delete[] column_ ;

    row_ = NULL ;
    column_ = NULL ;

    storage_ = NONE ;
    rows_are_stored_    = false ;
    columns_are_stored_ = false ;
    symmetric_storage_  = false ;
}

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseMatrix<T>::allocate(
    long m, long n, Storage storage, bool symmetric_storage
)
{
    m_ = m ;
    n_ = n ;
    symmetric_storage_ = symmetric_storage ;
    symmetric_tag_ = false ;
    storage_ = storage ;

    switch(storage)
    {
        case NONE:
            break ;
        case ROWS:
            rows_are_stored_    = true ;
            columns_are_stored_ = false ;
            break ;
        case COLUMNS:
            rows_are_stored_    = false ;
            columns_are_stored_ = true ;
            break ;
        case ROWS_AND_COLUMNS:
            rows_are_stored_    = true ;
            columns_are_stored_ = true ;
            break ;
    }

    if(rows_are_stored_)
    {
        row_ = new CNCSparseRowColumn<T>[m] ;
    }
    else
    {
        row_ = NULL ;
    }

    if(columns_are_stored_)
    {
        column_ = new CNCSparseRowColumn<T>[n] ;
    }
    else
    {
        column_ = NULL ;
    }
}

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseMatrix<T>::sort()
{
    if(rows_are_stored_)
    {
        for(long i = 0; i < m_; i++)
        {
            row(i).sort() ;
        }
    }

    if(columns_are_stored_)
    {
        for(long j = 0; j < n_; j++)
        {
            column(j).sort() ;
        }
    }
}

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseMatrix<T>::zero()
{
    if(rows_are_stored_)
    {
        for(long i = 0; i < m_; i++)
        {
            row(i).zero() ;
        }
    }

    if(columns_are_stored_)
    {
        for(long j = 0; j < n_; j++)
        {
            column(j).zero() ;
        }
    }
}

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseMatrix<T>::clear()
{
    if(rows_are_stored_)
    {
        for(long i = 0; i < m_; i++)
        {
            row(i).clear() ;
        }
    }

    if(columns_are_stored_)
    {
        for(long j = 0; j < n_; j++)
        {
            column(j).clear() ;
        }
    }
}

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseMatrix<T>::premult_transpose_add(CNCSparseMatrix &mat) const
{
    assert(rows_are_stored());

    for(int i = 0; i < m_; i++)
    {
        const CNCSparseRowColumn<T> &row = row_[i];

        int row_size = row.nb_coeffs();

        for(int j = 0; j < row_size; j++)
        {
            const CNCCoeff<T> &cj = row.coeff(j);
            const T &vj = cj.a;

            for(int k = 0; k < row_size; k++)
            {
                const CNCCoeff<T> &ck = row.coeff(k);
                mat.add(cj.index, ck.index, vj * ck.a);
            }
        }
    }
}

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseMatrix<T>::transpose_postmult(T *b, T *v) const
{
    if(rows_are_stored())
    {
        memset(v, 0, n_ * sizeof(T));

        for(int i = 0; i < m_; i++)
        {
            T val = b[i];
            const CNCSparseRowColumn<T> & curCol = row_[i];

            int col_size = curCol.nb_coeffs();

            for(int j = 0; j < col_size; j++)
            {
                const CNCCoeff<T> & cj = curCol.coeff(j);
                v[cj.index] += cj.a * val;
            }
        }
    }
    else
    {
        for(int i = 0; i < n_; i++)
        {
            const CNCSparseRowColumn<T> & curRow = column_[i];

            T val = 0.0f;
            int row_size = curRow.nb_coeffs();

            for(int j = 0; j < row_size; j++)
            {
                const CNCCoeff<T> & cj = curRow.coeff(j);
                val += cj.a * b[cj.index];
            }

            v[i] = val;
        }
    }
}

//---------------------------------------------------------------------------//
template <typename T>
void CNCSparseMatrix<T>::postmult(T *b, T *v) const
{
    if(rows_are_stored())
    {
        for(int i = 0; i < m_; i++)
        {
            const CNCSparseRowColumn<T> & row = row_[i];

            T val = 0.0f;
            int row_size = row.nb_coeffs();

            for(int j = 0; j < row_size; j++)
            {
                const CNCCoeff<T> & cj = row.coeff(j);
                val += cj.a * b[cj.index];
            }

            v[i] = val;
        }
    }
    else
    {
        memset(v, 0, m_ * sizeof(T));

        for(int i = 0; i < n_; i++)
        {
            T val = b[i];
            const CNCSparseRowColumn<T> & col = column_[i];

            int col_size = col.nb_coeffs();

            for(int j = 0; j < col_size; j++)
            {
                const CNCCoeff<T> &cj = col.coeff(j);
                v[cj.index] += cj.a * val;
            }
        }
    }
}

template <typename T>
void CNCSparseMatrix<T>::postmult_add(T *b, T *v) const
{
    if(rows_are_stored())
    {
        for(int i = 0; i < m_; i++)
        {
            const CNCSparseRowColumn<T> & row = row_[i];

            T val = 0.0f;
            int row_size = row.nb_coeffs();

            for(int j = 0; j < row_size; j++)
            {
                const CNCCoeff<T> & cj = row.coeff(j);
                val += cj.a * b[cj.index];
            }

            v[i] += val;
        }
    }
    else
    {
        for(int i = 0; i < n_; i++)
        {
            T val = b[i];
            const CNCSparseRowColumn<T> & col = column_[i];

            int col_size = col.nb_coeffs();

            for(int j = 0; j < col_size; j++)
            {
                const CNCCoeff<T> & cj = col.coeff(j);
                v[cj.index] += cj.a * val;
            }
        }
    }
}

//---------------------------------------------------------------------------//
template <typename T>
CNCSparseMatrix<T> & CNCSparseMatrix<T>::operator +=
(const CNCSparseMatrix<T> & mat)
{
    // set the fetch function
    typedef const CNCSparseRowColumn<T> &
    (CNCSparseMatrix<T>::*fetch_func)(long) const;
    fetch_func fetch = NULL;

    bool bRowStored = mat.rows_are_stored();

    if(bRowStored)
        fetch = &CNCSparseMatrix<T>::row;
    else
        fetch = &CNCSparseMatrix<T>::column;

    // fetch the coefficients
    bool bIsMatSymm = mat.is_symmetric();
    bool bIsThisSymm = is_symmetric();

    int count = bRowStored ? mat.m() : mat.n();

    for(int i = 0; i < count; i++)
    {
        const CNCSparseRowColumn<T> & vec = (mat.*fetch)(i);

        int nCoeff = vec.nb_coeffs();

        for(int j = 0; j < nCoeff; j++)
        {
            const CNCCoeff<T> &coeff = vec.coeff(j);
            T val = coeff.a;
            int ind = coeff.index;

            if(bIsMatSymm && i > ind)
            {
                add(i, ind, val);

                if(!bIsThisSymm)
                    add(ind, i, val);
            }
            else if(bRowStored)
                add(i, ind, val);
            else
                add(ind, i, val);
        }
    }

    return *this;
}


template <typename T>
CNCSparseMatrix<T> &CNCSparseMatrix<T>::operator -=
(const CNCSparseMatrix<T> &mat)
{
    assert(m_ == mat.m_ && n_ == mat.n_);

    if(mat.columns_are_stored() && columns_are_stored())
    {
        for(int j = 0; j < n_; j++)
        {
            const CNCSparseRowColumn<T> &srcCol = mat.column(j);
            CNCSparseRowColumn<T> &dstCol = column(j);

            int nCoeff = srcCol.nb_coeffs();

            for(int i = 0; i < nCoeff; i++)
            {
                const CNCCoeff<T> &curCoeff = srcCol.coeff(i);
                dstCol.add(curCoeff.index, -curCoeff.a);
            }
        }
    }

    return *this;
}

//---------------------------------------------------------------------------//
template <typename T>
CNCSparseMatrix<T> & CNCSparseMatrix<T>::operator *= (const T &s)
{
    // set the fetch function
    typedef CNCSparseRowColumn<T>&
    (CNCSparseMatrix<T>::*fetch_func)(long);
    fetch_func fetch = NULL;

    bool bRowStored = rows_are_stored();

    if(bRowStored)
        fetch = &CNCSparseMatrix<T>::row;
    else
        fetch = &CNCSparseMatrix<T>::column;

    // perform the multiplication
    int count = bRowStored ? m_ : n_;

    for(int i = 0; i < count; i++)
    {
        CNCSparseRowColumn<T> &vec = (this->*fetch)(i);

        int nCoeffs = vec.nb_coeffs();

        for(int j = 0; j < nCoeffs; j++)
        {
            CNCCoeff<T> &coeff = vec.coeff(j);
            coeff.a *= s;
        }
    }

    return *this;
}

template <typename T>
CNCSparseMatrix<T> & CNCSparseMatrix<T>::copy(const CNCSparseMatrix<T> & mat)
{
    // set the fetch function
    typedef CNCSparseRowColumn<T>&
    (CNCSparseMatrix<T>::*fetch_func)(long);
    fetch_func fetch = NULL;

    bool bRowStored = rows_are_stored();

    if(bRowStored)
        fetch = &CNCSparseMatrix<T>::row;
    else
        fetch = &CNCSparseMatrix<T>::column;

    return *this;
}

//---------------------------------------------------------------------------//
#endif

