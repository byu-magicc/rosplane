#ifndef NED_T_H
#define NED_T_H

#include <math.h>

namespace rosplane
{
struct NED_t
{
	double N;						// North (m)
	double E;						// East  (m)
	double D;						// Down  (m)
  NED_t()
  {
    N = 0.0f;
    E = 0.0f;
    D = 0.0f;
  }
  NED_t(double N_in, double E_in, double D_in)
  {
    N = N_in;
    E = E_in;
    D = D_in;
  }
	bool operator==(const NED_t s)
	{
		return N == s.N && E == s.E && D == s.D;
	}
	bool operator!=(const NED_t s)
	{
		return N != s.N || E != s.E || D != s.D;
	}
  NED_t operator+(const NED_t s)
  {
    NED_t n;
    n.N = N + s.N;
    n.E = E + s.E;
    n.D = D + s.D;
    return n;
  }
  NED_t operator-(const NED_t s)
  {
    NED_t n;
    n.N = N - s.N;
    n.E = E - s.E;
    n.D = D - s.D;
    return n;
  }
  double norm()
  {
    return sqrt(N*N + E*E + D*D);
  }
  NED_t normalize()
  {
    NED_t out;
    double magnitude = norm();
    if (magnitude <= 1.0e-9f)
      return out; // 0, 0, 0
    out.N = N/magnitude;
    out.E = E/magnitude;
    out.D = D/magnitude;
    return out;
  }
  double dot(NED_t in)
  {
    return N*in.N + E*in.E + D*in.D;
  }
  double getChi()
  {
    return atan2(E, N);
  }
  NED_t operator*(const double num)
  {
    NED_t n;
    n.N = N*num;
    n.E = E*num;
    n.D = D*num;
    return n;
  }
};
}

#endif
