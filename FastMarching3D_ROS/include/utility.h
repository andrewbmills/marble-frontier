#include <math.h>
#include <string>

double Utility_linear(double gain, double cost, double alpha);
double Utility_exponential(double gain, double cost, double lambda);
double Utility_efficiency(double gain, double cost);
double Utility(double gain, double cost, double param = 0.0, std::string type = "efficiency");

double Utility(double gain, double cost, double param = 0.0, std::string type = "efficiency")
{
  if (type == "linear") return utility_linear(gain, cost, param);
  else if (type == "exponential") return utility_exponential(gain, cost, param);
  else if (type == "efficiency") return utility_efficiency(gain, cost);
}

double Utility_linear(double gain, double cost, double alpha)
{
  return gain - alpha*cost;
}

double Utility_exponential(double gain, double cost, double lambda)
{
  return gain*std::exp(-lambda*cost);
}

double Utility_efficiency(double gain, double cost)
{
  if (cost > 0) {
    return gain/cost;
  }
  else {
    return 0.0;
  }
}