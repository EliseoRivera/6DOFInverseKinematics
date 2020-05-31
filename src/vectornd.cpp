#include "vectornd.h"
#include <iostream>
using namespace std;
vectornd::vectornd() {entry=nullptr;
dimension=0;
};
vectornd::vectornd(unsigned int  dim){
if (dim>0){
entry=new double[dim];
dimension=dim;
for (int i=0;i<dim;i++){
    entry[i]=0;

}

} else{
entry=nullptr;
dimension=0;
 cout<<"not initialized in constructor"<<endl;
 }
};
void vectornd::initZero(unsigned int  dim){
if (dimension==0&&dim>0){
entry=new double[dim];
dimension=dim;
for (int i=0;i<dim;i++){
    entry[i]=0;

}


} else
{

entry=nullptr;
dimension=0;
 cout<<"not initialized "<<endl;
}
};

vectornd::vectornd (const vectornd &p){

entry=new double[p.dimension];
dimension=p.dimension;
for (int i=0;i<dimension;i++){
    entry[i]=p.entry[i];

}


};
vectornd vectornd::operator+(const vectornd &p)const
{
 if (dimension==p.dimension){

        vectornd temp(dimension);

    for (int i=0;i<dimension;i++){
    temp.entry[i]=entry[i]+p.entry[i];

}
    return temp;

 } else {
    cout<<"different size in + operation "<<endl;

    return *this;
}


}

vectornd vectornd::operator-(const vectornd &p) const
{  if (dimension==p.dimension){

        vectornd temp(dimension);

    for (int i=0;i<dimension;i++){
    temp.entry[i]=entry[i]-p.entry[i];

}
    return temp;
 } else {
    cout<<"different size in - operation "<<endl;

    return *this;
}
 }
vectornd operator*(const double &s,const vectornd &p)
{

    vectornd temp(p.dimension);
    for (int i=0;i<p.dimension;i++){
    temp.entry[i]=s*p.entry[i];

}
     return temp;

}



double vectornd:: dotProduct(const vectornd &p)
{
     if (dimension==p.dimension){

      double temp=0;

    for (int i=0;i<dimension;i++){
    temp=temp+entry[i]*p.entry[i];

      }
    return temp;

      }
       else {
    cout<<"different size in dotProduct operation "<<endl;


}

}

double vectornd::magnitude()
{ return sqrt(this->dotProduct(*this));
};
vectornd&   vectornd::normalize(){

double a=magnitude();
if (a!=0){
      for (int i=0;i<dimension;i++){
    entry[i]=entry[i]/a;

      }
};
return *this;
 }


 vectornd   vectornd::unit(){

double a=magnitude();
if (a!=0){
        vectornd temp(dimension);
      for (int i=0;i<dimension;i++){
    temp.entry[i]=entry[i]/a;

      }
return temp;
}else
{
      cout<<" magnitude is zero, not normalized "<<endl;
    return *this;
}

 }
  vectornd&  vectornd::operator =( const vectornd &p){

     if (dimension==p.dimension){



    for (int i=0;i<dimension;i++){
    entry[i]=p.entry[i];

}
    return *this;
 }
 else {
    cout<<"different size in = operation "<<endl;

    return *this;
}



 }
vectornd::~vectornd(){
if (dimension>0) delete [] entry;
}
  void vectornd::mostrar(){

         for (int i=0;i<dimension;i++){
    cout<<" x" <<i<<" = "<<entry [i]<<endl;

      }


}
  double& vectornd:: operator [](unsigned int a){
  return entry[a];
  };
