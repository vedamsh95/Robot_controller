//
// Created by Timo on 07.01.2021.
//

#ifndef SDIR_CTRL2020_VECTOR_H
#define SDIR_CTRL2020_VECTOR_H

#pragma once

#include <cstdint>


template<class T, int N>
class Vector{
public:
    Vector() : data() {}

    T& operator[](int i)
    {
        return this->data[i];
    }


    Vector operator- (Vector<T,N>& vec) const
    {
        Vector<T,N> tmpVec;
        for(int i =0; i < N; i++)
        {
            tmpVec[i] = this->data[i] -vec[i];
        }
        return tmpVec;
    }

    Vector operator-()
    {
        Vector<T,N> tmpVec;
        for(int i=0; i < N; i++)
        {
            tmpVec[i] = this->data[i] * (-1);
        }
        return tmpVec;
    }


    Vector operator+ (Vector<T,N>& vec) const
    {
        Vector<T,N> tmpVec;
        for(int i =0; i < N; i++)
        {
            tmpVec[i] = this->data[i] +vec[i];
        }
        return tmpVec;
    }

    void output() const{
        if(N == 3){
            std::cout << "[" << this->data[0] << "," << this->data[1] << "," << this->data[2] << "]" << std::endl;
        }
        else {
            for (int i = 0; i < N; ++i) {
                std::cout << this->data[i] << std::endl;
            }
        }
    }

private:
    T data[N];
};



template <class T, int N>
Vector<T, N> operator*(T scal, Vector<T, N> &vec) {
    Vector<T, N> tmpVec;
    for (int i = 0; i < N; i++)
        tmpVec[i] = scal*vec[i];
    return tmpVec;
}


template <class T, int N>
Vector<T, N> operator*(Vector<T, N> &vec, T scal) {
    Vector<T, N> tmpVec;
    for (int i = 0; i < N; i++)
        tmpVec[i] = scal*vec[i];
    return tmpVec;
}

template <class T, int N>
Vector<T, N> operator+(Vector<T, N> &vec, T scal) {
    Vector<T, N> tmpVec;
    for (int i = 0; i < N; i++)
        tmpVec[i] = scal+vec[i];
    return tmpVec;
}

template <class T, int N>
Vector<T, N> operator+(T scal, Vector<T, N> &vec) {
    Vector<T, N> tmpVec;
    for (int i = 0; i < N; i++)
        tmpVec[i] = scal+vec[i];
    return tmpVec;
}


template <class T, int N>
Vector<T, N> operator+(Vector<T, N> &vec1, Vector<T, N> &vec2) {
    Vector<T, N> tmpVec;
    for (int i = 0; i < N; i++)
        tmpVec[i] = vec1[i]+vec2[i];
    return tmpVec;
}

#endif //SDIR_CTRL2020_VECTOR_H
