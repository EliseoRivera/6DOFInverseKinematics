#include "Matrix.h"
using namespace std;
Matrix::Matrix(vector3d v){
int nn=3; int mm=1;

if (nn>0&&mm>0){
  aij=new double*[nn];
  for(int i = 0; i < nn; i++)  aij[i] = new double[mm];
  n=nn;
  m=mm;

  for( int i = 0; i < n; i++)
    for(int j = 0; j < m; j++){
    //    coutincorrect <<"introduce ij ("<<i+1 <<","<<j+1<<")"<<endl;
           // cin>>ij(i,j);
aij[i][j]=0;
    };
}
ij(0,0)=v.x;
ij(1,0)=v.y;
ij(2,0)=v.z;
}
Matrix::Matrix(vector3d v,float w){
int nn=4; int mm=1;

if (nn>0&&mm>0){
  aij=new double*[nn];
  for(int i = 0; i < nn; i++)  aij[i] = new double[mm];
  n=nn;
  m=mm;

  for( int i = 0; i < n; i++)
    for(int j = 0; j < m; j++){
    //    coutincorrect <<"introduce ij ("<<i+1 <<","<<j+1<<")"<<endl;
           // cin>>ij(i,j);
aij[i][j]=0;
    };
}

ij(0,0)=v.x;
ij(1,0)=v.y;
ij(2,0)=v.z;
ij(3,0)=1;
}
double Matrix::cofactor (int i, int j){
double c;
c=pow (-1,i+j)*Mij(i,j).determinante();  //cofactor
return c;
};
Matrix Matrix::inversa(){
double d=(*this).determinante();

if (d==0) { Matrix B(1,1); cout<<" no existe la inversa"<<endl; return B;}
 if(n>2) {
    Matrix B(n,n);
     for( int i = 0; i < n; i++)
    for(int j = 0; j < n; j++){
         B.aij[j][i]=(1.0/d)*cofactor(i,j);


    }
    return B;

}
else if(n==2) {
    Matrix B(n,n);
     B.aij[0][0]=(1.0/d)*aij[1][1];

      B.aij[0][1]=-(1.0/d)*aij[0][1];
       B.aij[1][0]=-(1.0/d)*aij[1][0];
      B.aij[1][1]=(1.0/d)*aij[0][0];
    return B;

}
else if(n==1) {
    Matrix B(1,1);
     B.aij[0][0]=1/aij[0][0];


    return B;

}

};
    int Matrix::size () const{return n*m;};
    Matrix Matrix::Mij(int a,int b){ //menor de una matriz
        if (n==m&&size()>0){ // eliminar fila a y columna b

    int s=n-1;
    Matrix B(s,s); //matriz de tamaño reducido
    for (int i=0; i<a;i++)
    for (int j=0; j<b;j++)

    {
      B.aij[i][j]=aij[i][j]; //primera parte
    }

      for (int i=0; i<a;i++)
    for (int j=b+1; j<m;j++)

    {
      B.aij[i][j-1]=aij[i][j];  //se recorren las columnas
    }

        for (int i=a+1; i<n;i++)
    for (int j=0; j<b;j++)

    {
      B.aij[i-1][j]=aij[i][j];  //se recorren las columnas
    }


        for (int i=a+1; i<n;i++)
    for (int j=b+1; j<m;j++)

    {
      B.aij[i-1][j-1]=aij[i][j];   //se recorren las columnas
    }
    return B;
}
};
vector3d Matrix::operator *(const vector3d &P){//operacion multiplicacion por punto
Matrix A(3,1);
A.aij[0][0]=P.x;
A.aij[1][0]=P.y;
A.aij[2][0]=P.z;

Matrix T((*this)*A);

vector3d Pr;
Pr.x=T.aij[0][0];
Pr.y=T.aij[1][0];
Pr.z=T.aij[2][0];
return Pr;
}
Matrix::Matrix(int nn,int mm){ //constructor a partir de n y m
if (nn>0&&mm>0){
  aij=new double*[nn];
  for(int i = 0; i < nn; i++)  aij[i] = new double[mm];
  n=nn;
  m=mm;

  for( int i = 0; i < n; i++)
    for(int j = 0; j < m; j++){
    //    coutincorrect <<"introduce ij ("<<i+1 <<","<<j+1<<")"<<endl;
           // cin>>ij(i,j);
aij[i][j]=0;
    };
} ;   //numero de filas
//coutincorrect<<"\n"<<endl;
};
Matrix::Matrix(){



n=m=0;
  //numero de filas

};
void Matrix::zero(int nn, int mm){
     n=nn;
        m=mm;
     //constructor de mat


if (n>0&&m>0){



  aij=new double*[n];
  for(int i = 0; i < n; i++)  aij[i] = new double[m];


  for( int i = 0; i < n; i++)
    for(int j = 0; j < m; j++){

        ij(i,j)=0.0;;

    };
} ;   //numero de filas
//coutincorrect<<"\n"<<endl;
};
 void Matrix::resetIdentity(){

  for( int i = 0; i < n; i++)
    for(int j = 0; j < m; j++){
if(  i==j)ij(i,j)=1.0;
else ij(i,j)=0.0;

    };
 }

void Matrix::identity(int nn){
     n=nn;
        m=nn;
     //constructor de mat


if (n>0&&m>0){



  aij=new double*[n];
  for(int i = 0; i < n; i++)  aij[i] = new double[m];


  for( int i = 0; i < n; i++)
    for(int j = 0; j < m; j++){
if(  i==j)ij(i,j)=1.0;
else ij(i,j)=0.0;

    };
} ;   //numero de filas
//coutincorrect<<"\n"<<endl;
};

void Matrix::definir(){
    cout <<"Creando Matriz "<<endl;
     cout <<"introduce el numero de filas "<<endl;
     cin>> n;
         cout <<"introduce  el numero de columnas "<<endl;
              cin>> m;
     //constructor de mat


if (n>0&&m>0){



  aij=new double*[n];
  for(int i = 0; i < n; i++)  aij[i] = new double[m];


  for( int i = 0; i < n; i++)
    for(int j = 0; j < m; j++){
        cout<<"introduce ij ("<<i+1 <<","<<j+1<<")"<<endl;
        cin>>ij(i,j);

    };
} ;   //numero de filas
//coutincorrect<<"\n"<<endl;
};
void Matrix::mostrar(){
  if (this->n>0&&this->m>0){


  cout<<"\n"<<endl;
for( int i = 0; i < n; i++){
    for( int j = 0; j < m; j++){

      cout<<ij(i,j)<<"  "<<flush;
    }
    cout<<"\n"<<endl;
    }
}
}
double& Matrix::ij(int i,int j){ //devuelve la direccion
if (i>=0&&j>=0&&i<n&&j<m) return aij[i][j]; //c++ numera las ijs de una matrix a partir de cero, nosotros a partir de 1

};
Matrix::Matrix(const Matrix &B){
if(B.size()!=0){
        n=B.n;
        m=B.m;


        aij=new double*[n];
  for(int i = 0; i < n; i++)  aij[i] = new double[m];

    for( int i = 0; i < n; i++)
    for( int j = 0; j < m; j++){

       aij[i][j]=B.aij[i][j];

    }

}
};
Matrix Matrix::operator +( const Matrix &A){
if (n>0&&m>0){
 if (n==A.n&&m==A.m){

Matrix B(n,m);

for( int i = 0; i < n; i++)
    for( int j = 0; j < m; j++){

        B.aij[i][j]= aij[i][j]+A.aij[i][j];

    }

      return B;
    }
    else{Matrix B(1,1); cout<<" Tamaños diferentes en suma"<<endl;  return B;};

    }


};
Matrix& Matrix::operator =( const Matrix &A){
if (A.n>0&&A.m>0&&n==A.n&&m==A.m){

Matrix B(n,m);
for( int i = 0; i < n; i++)
    for(int j = 0; j < m; j++){

       aij[i][j]=A.aij[i][j];
    }
      return *this;

     }   else{Matrix B(1,1); cout<<" Tamaños diferentes en igual"<<endl;  return *this;}

};
Matrix Matrix::operator -( Matrix A){
if (this->n>0&&this->m>0){
 if (this->n==A.n&&this->m==A.m){
Matrix B(n,m);
for( int i = 0; i < this->n; i++)
    for(int j = 0; j < this->m; j++){

        B.ij(i,j)= ij(i,j)-A.ij(i,j);
    }
      return B;
    }    else{Matrix B(1,1); cout<<" Tamaños diferentes en resta"<<endl;  return B;}
}

};
Matrix Matrix::operator *( Matrix B){ //multiplicaciion de matrices
if (size()>0&&B.size()>0){
 if (m==B.n){
Matrix A(n,B.m);

for( int i = 0; i < n; i++)
    for(int j = 0; j < B.m; j++){
            float s=0;
    for(int k=0;k<m;k++){s=s+ij(i,k)*B.ij(k,j); }
        A.ij(i,j)= s;
    }
      return A;
    }    else{Matrix B(1,1); cout<<" Tamaños diferentes en multiplicacion"<<endl;  return B;} }


};
Matrix operator*(double tt, Matrix &A){
if (A.n>0&&A.m>0){

Matrix B(A.n,A.m);
for(int i = 0; i < A.n; i++)
    for( int j = 0; j < A.m; j++){

        B.ij(i,j)= tt*A.ij(i,j);
    }


    return B;
     }


};
Matrix::~Matrix(){


			if (n>0){
				for (int i = 0; i<n; i++){
					delete[] aij[i];  //se elimnan las columnas
				}
				delete[] aij; //eliminar las filas
				aij = NULL;
			}
	}

double Matrix::determinante(){  //se define en forma recursiva
double d=0;
if (n!=m) {cout<<"la matriz no es cuadrada"<<endl; return 0; }
 if (n>2){
for (int j=0; j<m;j++){

    d=d+aij[0][j]*pow (-1,j)*Mij(0,j).determinante();

}
return d;
} else if (n==2) {
    d=aij[0][0]*aij[1][1]-aij[0][1]*aij[1][0];
    return d;

} else if(n==1){

return aij[0][0];
};

}

double& Matrix::entry(int i, int j){
return aij[i][j];
}
Matrix Matrix::crossproduct(Matrix &B){
   Matrix v(4,1);

 v.entry(0,0)=entry(1,0)*B.entry(2,0)-entry(2,0)*B.entry(1,0);
 v.entry(1,0)=entry(2,0)*B.entry(0,0)-entry(0,0)*B.entry(2,0);
 v.entry(2,0)=entry(0,0)*B.entry(1,0)-entry(1,0)*B.entry(0,0);
   v.entry(3,0)=0;
   return v;
};

   double Matrix::magnitud(){
  double m;
m=sqrt(entry(0,0)*entry(0,0)+entry(1,0)*entry(1,0)+entry(2,0)*entry(2,0));
  return m;
   };


double Matrix::angle (Matrix  &B){
  double a;
a =acos((entry(0,0)*B.entry(0,0)+entry(1,0)*B.entry(1,0)+entry(2,0)*B.entry(2,0))/(magnitud()*B.magnitud()));
  return a;
   };


 Matrix  Matrix::normalizar (){
 Matrix u(4,1);
 u=(1.0/magnitud())*(*this);

 u.entry(3,0)=0;
 return u;
 };

  Matrix  Matrix::Transpuesta(){
 Matrix T(m,n);
 for ( int i=0;i<m;i++)
 for ( int j=0;j<n;j++){


    T.entry(i,j)=entry(j,i);

 }

 return T;
 };

