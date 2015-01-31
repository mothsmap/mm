#ifndef __rl__utility__hh__
#define __rl__utility__hh__

#include "CNC/cnc_sparse_matrix.h"
#include <vector>

//! Reinforcement Learning Algorithm namespace.
//! Utility functions for Reinforcement Learning.
class RLUtility {
public:
    RLUtility() {}
    ~RLUtility() {}
    
    //! Get first max element in the array.
    /*!
     \ par*am array the array.
     \param n the array size.
     \return the first max element in the array.
     \sa ArgMax(), MaxAll(), ArgMaxAll()
     */
    static double Max(double* array, int n);
    static double Max(const CNCSparseRowColumn<double>& row);
    
    //! Get first max element in the sub-array of an array.
    /*!
     \ par*am array the array.
     \param index the sub-array's index in the array.
     \return the first max element in the sub-array.
     */
    static double Max(double* array, std::vector<int> index);
    
    //! Get first max element's index in the array.
    /*!
     \ par*am array the array.
     \param n the array size.
     \return the first max element in the array.
     */
    static int ArgMax(double* array, int n);
    static int ArgMax(const CNCSparseRowColumn<double>& row);
    
    //! Get first max element's index in the sub-array of an array.
    /*!
     \ par*am array the array.
     \param index the sub-array's index in the array.
     \return the first max element's index in the sub-array.
     */
    static int ArgMax(double* array, std::vector<int> index);
    
    //! Get all the max elements' indexes in the array.
    /*!
     \ par*am array the array.
     \param n array size.
     \return all the max elements' indexes in the array.
     */
    static std::vector<int> ArgMaxAll(double* array, int n);
    static std::vector<int> ArgMaxAll(const CNCSparseRowColumn<double>& row);
    //! Get all the max elements' indexes in the sub-array of an array.
    /*!
     \ par*am array the array.
     \param index the sub-array's index in the array.
     \return all the max elements' indexes in the sub-array of an array.
     */
    static std::vector<int> ArgMaxAll(double* array, std::vector<int> index);
    
    //! Generate an int in [0, A)
    static int Rand0A(int A);
    
    //! Generate an int in [A, B)
    static int RandAB(int A, int B);
    
    //! Generate a double in [0, 1]
    static double RandUnit();
};

#endif