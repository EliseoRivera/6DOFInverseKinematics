#ifndef vectornd_H
#define vectornd_H
#include <math.h>

class vectornd
{
    public:
    double* entry;
    vectornd(unsigned int dim);
    vectornd();
    void initZero(unsigned int  dim);
    void resetZero();
    ~vectornd();
    vectornd (const vectornd &p);
    vectornd operator+(const vectornd &p)const;
    vectornd operator-(const vectornd &p)const;
    friend vectornd operator*(const double &s, const vectornd &p);

    double dotProduct(const vectornd &p);
    double magnitude() ;
    vectornd&   normalize();
      vectornd  unit();
    vectornd&  operator =( const vectornd &B);
      double& operator [](unsigned int a);
  unsigned int dimension;
    void mostrar();
};
#endif // vector3d_H
