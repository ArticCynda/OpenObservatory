// poly.cpp : solution of cubic and quartic equation
// (c) Khashin S.I. http://math.ivanovo.ac.ru/dalgebra/Khashin/index.html
// khash2 (at) gmail.com
//
#include <math.h>
#include "SI114X.h"





//#include "poly34.h"     // solution of cubic and quartic equation


/*






//-----------------------------------------------------------------------------
#define F5(t) (((((t+a)*t+b)*t+c)*t+d)*t+e)
//-----------------------------------------------------------------------------
double SI114X::SolveP5_1(double a,double b,double c,double d,double e)	// return real root of x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
{
	int cnt;
	if( fabs(e)<eps ) return 0;

	double brd =  fabs(a);			// brd - border of real roots
	if( fabs(b)>brd ) brd = fabs(b);
	if( fabs(c)>brd ) brd = fabs(c);
	if( fabs(d)>brd ) brd = fabs(d);
	if( fabs(e)>brd ) brd = fabs(e);
	brd++;							// brd - border of real roots

	double x0, f0;					// less, than root
	double x1, f1;					// greater, than root
	double x2, f2, f2s;				// next values, f(x2), f'(x2)
	double dx;

	if( e<0 ) { x0 =   0; x1 = brd; f0=e; f1=F5(x1); x2 = 0.01*brd; }
	else	  { x0 =-brd; x1 =   0; f0=F5(x0); f1=e; x2 =-0.01*brd; }

	if( fabs(f0)<eps ) return x0;
	if( fabs(f1)<eps ) return x1;

	// now x0<x1, f(x0)<0, f(x1)>0
	// Firstly 5 bisections
	for( cnt=0; cnt<5; cnt++)
	{
		x2 = (x0 + x1)/2;			// next point
		f2 = F5(x2);				// f(x2)
		if( fabs(f2)<eps ) return x2;
		if( f2>0 ) { x1=x2; f1=f2; }
		else       { x0=x2; f0=f2; }
	}

	// At each step:
	// x0<x1, f(x0)<0, f(x1)>0.
	// x2 - next value
	// we hope that x0 < x2 < x1, but not necessarily
	do {
		cnt++;
		if( x2<=x0 || x2>= x1 ) x2 = (x0 + x1)/2;	// now  x0 < x2 < x1
		f2 = F5(x2);								// f(x2)
		if( fabs(f2)<eps ) return x2;
		if( f2>0 ) { x1=x2; f1=f2; }
		else       { x0=x2; f0=f2; }
		f2s= (((5*x2+4*a)*x2+3*b)*x2+2*c)*x2+d;		// f'(x2)
		if( fabs(f2s)<eps ) { x2=1e99; continue; }
		dx = f2/f2s;
		x2 -= dx;
	} while(fabs(dx)>eps);
	return x2;
} // SolveP5_1(double a,double b,double c,double d,double e)	// return real root of x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
//-----------------------------------------------------------------------------
int   SI114X::SolveP5(double *x,double a,double b,double c,double d,double e)	// solve equation x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
{
	double r = x[0] = SolveP5_1(a,b,c,d,e);
	double a1 = a+r, b1=b+r*a1, c1=c+r*b1, d1=d+r*c1;
	return 1+SolveP4(x+1, a1,b1,c1,d1);
} // SolveP5(double *x,double a,double b,double c,double d,double e)	// solve equation x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
//-----------------------------------------------------------------------------
// let f(x ) = a*x^2 + b*x + c and 
//     f(x0) = f0,
//     f(x1) = f1,
//     f(x2) = f3
// Then r1, r2 - root of f(x)=0.
// Returns 0, if there are no roots, else return 2.
int SI114X::Solve2( double x0, double x1, double x2, double f0, double f1, double f2, double &r1, double &r2)
{
	double w0 = f0*(x1-x2);
	double w1 = f1*(x2-x0);
	double w2 = f2*(x0-x1);
	double a1 =  w0         + w1         + w2;
	double b1 = -w0*(x1+x2) - w1*(x2+x0) - w2*(x0+x1);
	double c1 =  w0*x1*x2   + w1*x2*x0   + w2*x0*x1;
	double Di =  b1*b1 - 4*a1*c1;	// must be>0!
	if( Di<0 ) { r1=r2=1e99; return 0; }
	Di =  sqrt(Di);
	r1 =  (-b1 + Di)/2/a1;
	r2 =  (-b1 - Di)/2/a1;
	return 2;
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
*/
