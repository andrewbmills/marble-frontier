#include <math.h>
#include <string>

double UtilityLinear(double gain, double cost, double alpha);
double UtilityExponential(double gain, double cost, double lambda);
double UtilityEfficiency(double gain, double cost);
double Utility(double gain, double cost, double param, std::string type);

double CostFromUtilityGainLinear(double gain, double utility, double alpha);
double CostFromUtilityGainExponential(double gain, double utility, double lambda);
double CostFromUtilityGainEfficiency(double gain, double utility);
double CostFromUtilityGain(double gain, double utility, double param, std::string type);

double Utility(double gain, double cost, double param = 0.0, std::string type = "efficiency")
{
  if (type == "linear") return UtilityLinear(gain, cost, param);
  else if (type == "exponential") return UtilityExponential(gain, cost, param);
  else if (type == "efficiency") return UtilityEfficiency(gain, cost);
}

double UtilityLinear(double gain, double cost, double alpha)
{
  return gain - alpha*cost;
}

double UtilityExponential(double gain, double cost, double lambda)
{
  return gain*std::exp(-lambda*cost);
}

double UtilityEfficiency(double gain, double cost)
{
  if (cost > 0) {
    return gain/cost;
  }
  else {
    return 0.0;
  }
}

double CostFromUtilityGain(double gain, double utility, double param = 0.0, std::string type = "efficiency")
{
  if (type == "linear") return CostFromUtilityGainLinear(gain, utility, param);
  else if (type == "exponential") return CostFromUtilityGainExponential(gain, utility, param);
  else if (type == "efficiency") return CostFromUtilityGainEfficiency(gain, utility);
}

double CostFromUtilityGainLinear(double gain, double utility, double alpha)
{
  return (gain - utility)/alpha;
}

double CostFromUtilityGainExponential(double gain, double utility, double lambda)
{
  return -std::log(utility/gain)/lambda;
}

double CostFromUtilityGainEfficiency(double gain, double utility)
{
  if (utility > 0) {
    return gain/utility;
  }
  else {
    return 1e10;
  }
}