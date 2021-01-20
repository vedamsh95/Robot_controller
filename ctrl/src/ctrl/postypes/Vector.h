//
// Created by Timo on 07.01.2021.
//

#ifndef SDIR_CTRL2020_VECTOR_H
#define SDIR_CTRL2020_VECTOR_H

#pragma once

#include <cstdint>
#include <json.h>


template<class T, int N>
class Vector{
public:
    Vector(Json::Value value){
        this->data[0] = value["x"].asDouble();
        this->data[1] = value["y"].asDouble();
        this->data[2] = value["z"].asDouble();
    }

    Vector() : data() {}


    T& operator[](int i)
    {
        return this->data[i];
    }



    Vector<T,N>& operator+= (Vector<T,N>& vec)
    {
        for(int i =0; i < N; i++)
        {
            this->data[i] += vec[i];
        }
        return *this;
    }

    Vector<T,N> operator+ (Vector<T,N>& vec)
    {
        Vector<T,N> tmpVec;
        for(int i =0; i < N; i++)
        {
            tmpVec[i] = this->data[i] + vec[i];
        }
        return tmpVec;
    }

    Vector<T,N>& operator-= (Vector<T,N>& vec )
    {
        for(int i =0; i < N; i++)
        {
            this->data[i] -= vec[i];
        }
        return *this;
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

    Vector<T,N>& operator*= (const T scal){
        for(int i =0; i < N; i++)
        {
            this->data[i] *= scal;
        }
        return *this;
    }

    /*
    Vector<T,N>& operator=(const Vector<T,N> &vec)
    {
        if(this != &vec)
        {
            for(int i =0; i < N; i++)
            {
                this->data[i] = vec[i];
            }
        }
    }
*/
    bool operator==(Vector<T,N>& vec) {
        bool tmp = true;
        for (int i = 0; i < N; ++i) {
            if (this->data[i] != vec[i])
            {
                tmp = false;
                break;
            }
        }
        return tmp;
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
    void output_for_mac() const{
        if(N == 3){
            std::cout << this->data[0] << "," << this->data[1] << "," << this->data[2] << std::endl;
        }
        else {
            for (int i = 0; i < N; ++i) {
                std::cout << this->data[i] << std::endl;
            }
        }
    }

    double length(){
        double tmp = 0;
        for(int i = 0; i<N; i++)
        {
            tmp += (this->data[i] * this->data[i]);
        }
        return (sqrt(tmp));
    }

    double dot_product(Vector<T,N> &vec){
        double tmp = 0;
        for (int i = 0; i < N; ++i) {
            tmp += this->data[i] * vec[i];
        }
        return tmp;
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

template <class T, int N>
Vector<T, N> operator-(Vector<T, N> &vec1, Vector<T, N> &vec2) {
    Vector<T, N> tmpVec;
    for (int i = 0; i < N; i++)
        tmpVec[i] = vec1[i]-vec2[i];
    return tmpVec;
}

#endif //SDIR_CTRL2020_VECTOR_H
