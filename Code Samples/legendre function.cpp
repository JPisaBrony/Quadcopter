#include <math.h>
#include <Arduino.h>

#define MAX_LEGENDRE_ORDER 13
#define MAX_LEGENDRE_DEGREE 13
#define EARTH_RADIUS 6371.2L
#define MAJOR_AXIS  6378.137L
#define INVERSE_FLATTENING 298.257223563L
#define HALF_DELTA_RADIUS 0.0005
#define HALF_DELTA_THETA 0.0005
#define HALF_DELTA_PHI 0.0005

long double legendreTheta;//stores the last theta value used in the legendreCos function
long double legendreValue[MAX_LEGENDRE_ORDER + 1][MAX_LEGENDRE_DEGREE + 1];//stores the results of the legendreCos function to minimize repeated calculations
char legendreBoolean[MAX_LEGENDRE_ORDER + 1][MAX_LEGENDRE_DEGREE + 1];//flags if the value is current or not, if not current it will be recalculated as needed

//struct for IGRF models 
typedef struct IGRF
{
	unsigned char maxDegree;
	double baseYear;
	double gCoefficents[MAX_LEGENDRE_ORDER + 1][MAX_LEGENDRE_DEGREE + 1];
	double hCoefficents[MAX_LEGENDRE_ORDER + 1][MAX_LEGENDRE_DEGREE + 1];
	double gSV[MAX_LEGENDRE_ORDER + 1][MAX_LEGENDRE_DEGREE + 1];
	double hSV[MAX_LEGENDRE_ORDER + 1][MAX_LEGENDRE_DEGREE + 1];
} IGRF;

IGRF getIGRF11Coefficents()//data from http://www.ngdc.noaa.gov/IAGA/vmod/igrf.html
{
	IGRF igrf11;
	
	igrf11.maxDegree = 13;
	
	igrf11.baseYear = 2010.0;
	
	igrf11.gCoefficents[0][1] = -29496.5;
	igrf11.gCoefficents[1][1] = -1585.9;
	igrf11.hCoefficents[1][1] = 4945.1;
	igrf11.gCoefficents[0][2] = -2396.6;
	igrf11.gCoefficents[1][2] = 3026.0;
	igrf11.hCoefficents[1][2] = -2707.7;
	igrf11.gCoefficents[2][2] = 1668.6;
	igrf11.hCoefficents[2][2] = -575.4;
	igrf11.gCoefficents[0][3] = 1339.7;
	igrf11.gCoefficents[1][3] = -2326.3;
	igrf11.hCoefficents[1][3] = -160.5;
	igrf11.gCoefficents[2][3] = 1231.7;
	igrf11.hCoefficents[2][3] = 251.7;
	igrf11.gCoefficents[3][3] = 634.2;
	igrf11.hCoefficents[3][3] = -536.8;
	igrf11.gCoefficents[0][4] = 912.6;
	igrf11.gCoefficents[1][4] = 809.0;
	igrf11.hCoefficents[1][4] = 286.4;
	igrf11.gCoefficents[2][4] = 166.6;
	igrf11.hCoefficents[2][4] = -211.2;
	igrf11.gCoefficents[3][4] = -357.1;
	igrf11.hCoefficents[3][4] = 164.4;
	igrf11.gCoefficents[4][4] = 89.7;
	igrf11.hCoefficents[4][4] = -309.2;
	igrf11.gCoefficents[0][5] = -231.1;
	igrf11.gCoefficents[1][5] = 357.2;
	igrf11.hCoefficents[1][5] = 44.7;
	igrf11.gCoefficents[2][5] = 200.3;
	igrf11.hCoefficents[2][5] = 188.9;
	igrf11.gCoefficents[3][5] = -141.2;
	igrf11.hCoefficents[3][5] = -118.1;
	igrf11.gCoefficents[4][5] = -163.1;
	igrf11.hCoefficents[4][5] = 0.1;
	igrf11.gCoefficents[5][5] = -7.7;
	igrf11.hCoefficents[5][5] = 100.9;
	igrf11.gCoefficents[0][6] = 72.8;
	igrf11.gCoefficents[1][6] = 68.6;
	igrf11.hCoefficents[1][6] = -20.8;
	igrf11.gCoefficents[2][6] = 76.0;
	igrf11.hCoefficents[2][6] = 44.2;
	igrf11.gCoefficents[3][6] = -141.4;
	igrf11.hCoefficents[3][6] = 61.5;
	igrf11.gCoefficents[4][6] = -22.9;
	igrf11.hCoefficents[4][6] = -66.3;
	igrf11.gCoefficents[5][6] = 13.1;
	igrf11.hCoefficents[5][6] = 3.1;
	igrf11.gCoefficents[6][6] = -77.9;
	igrf11.hCoefficents[6][6] = 54.9;
	igrf11.gCoefficents[0][7] = 80.4;
	igrf11.gCoefficents[1][7] = -75.0;
	igrf11.hCoefficents[1][7] = -57.8;
	igrf11.gCoefficents[2][7] = -4.7;
	igrf11.hCoefficents[2][7] = -21.2;
	igrf11.gCoefficents[3][7] = 45.3;
	igrf11.hCoefficents[3][7] = 6.6;
	igrf11.gCoefficents[4][7] = 14.0;
	igrf11.hCoefficents[4][7] = 24.9;
	igrf11.gCoefficents[5][7] = 10.4;
	igrf11.hCoefficents[5][7] = 7.0;
	igrf11.gCoefficents[6][7] = 1.6;
	igrf11.hCoefficents[6][7] = -27.7;
	igrf11.gCoefficents[7][7] = 4.9;
	igrf11.hCoefficents[7][7] = -3.4;
	igrf11.gCoefficents[0][8] = 24.3;
	igrf11.gCoefficents[1][8] = 8.2;
	igrf11.hCoefficents[1][8] = 10.9;
	igrf11.gCoefficents[2][8] = -14.5;
	igrf11.hCoefficents[2][8] = -20.0;
	igrf11.gCoefficents[3][8] = -5.7;
	igrf11.hCoefficents[3][8] = 11.9;
	igrf11.gCoefficents[4][8] = -19.3;
	igrf11.hCoefficents[4][8] = -17.4;
	igrf11.gCoefficents[5][8] = 11.6;
	igrf11.hCoefficents[5][8] = 16.7;
	igrf11.gCoefficents[6][8] = 10.9;
	igrf11.hCoefficents[6][8] = 7.1;
	igrf11.gCoefficents[7][8] = -14.1;
	igrf11.hCoefficents[7][8] = -10.8;
	igrf11.gCoefficents[8][8] = -3.7;
	igrf11.hCoefficents[8][8] = 1.7;
	igrf11.gCoefficents[0][9] = 5.4;
	igrf11.gCoefficents[1][9] = 9.4;
	igrf11.hCoefficents[1][9] = -20.5;
	igrf11.gCoefficents[2][9] = 3.4;
	igrf11.hCoefficents[2][9] = 11.6;
	igrf11.gCoefficents[3][9] = -5.3;
	igrf11.hCoefficents[3][9] = 12.8;
	igrf11.gCoefficents[4][9] = 3.1;
	igrf11.hCoefficents[4][9] = -7.2;
	igrf11.gCoefficents[5][9] = -12.4;
	igrf11.hCoefficents[5][9] = -7.4;
	igrf11.gCoefficents[6][9] = -0.8;
	igrf11.hCoefficents[6][9] = 8.0;
	igrf11.gCoefficents[7][9] = 8.4;
	igrf11.hCoefficents[7][9] = 2.2;
	igrf11.gCoefficents[8][9] = -8.4;
	igrf11.hCoefficents[8][9] = -6.1;
	igrf11.gCoefficents[9][9] = -10.1;
	igrf11.hCoefficents[9][9] = 7.0;
	igrf11.gCoefficents[0][10] = -2.0;
	igrf11.gCoefficents[1][10] = -6.3;
	igrf11.hCoefficents[1][10] = 2.8;
	igrf11.gCoefficents[2][10] = 0.9;
	igrf11.hCoefficents[2][10] = -0.1;
	igrf11.gCoefficents[3][10] = -1.1;
	igrf11.hCoefficents[3][10] = 4.7;
	igrf11.gCoefficents[4][10] = -0.2;
	igrf11.hCoefficents[4][10] = 4.4;
	igrf11.gCoefficents[5][10] = 2.5;
	igrf11.hCoefficents[5][10] = -7.2;
	igrf11.gCoefficents[6][10] = -0.3;
	igrf11.hCoefficents[6][10] = -1.0;
	igrf11.gCoefficents[7][10] = 2.2;
	igrf11.hCoefficents[7][10] = -4.0;
	igrf11.gCoefficents[8][10] = 3.1;
	igrf11.hCoefficents[8][10] = -2.0;
	igrf11.gCoefficents[9][10] = -1.0;
	igrf11.hCoefficents[9][10] = -2.0;
	igrf11.gCoefficents[10][10] = -2.8;
	igrf11.hCoefficents[10][10] = -8.3;
	igrf11.gCoefficents[0][11] = 3.0;
	igrf11.gCoefficents[1][11] = -1.5;
	igrf11.hCoefficents[1][11] = 0.1;
	igrf11.gCoefficents[2][11] = -2.1;
	igrf11.hCoefficents[2][11] = 1.7;
	igrf11.gCoefficents[3][11] = 1.6;
	igrf11.hCoefficents[3][11] = -0.6;
	igrf11.gCoefficents[4][11] = -0.5;
	igrf11.hCoefficents[4][11] = -1.8;
	igrf11.gCoefficents[5][11] = 0.5;
	igrf11.hCoefficents[5][11] = 0.9;
	igrf11.gCoefficents[6][11] = -0.8;
	igrf11.hCoefficents[6][11] = -0.4;
	igrf11.gCoefficents[7][11] = 0.4;
	igrf11.hCoefficents[7][11] = -2.5;
	igrf11.gCoefficents[8][11] = 1.8;
	igrf11.hCoefficents[8][11] = -1.3;
	igrf11.gCoefficents[9][11] = 0.2;
	igrf11.hCoefficents[9][11] = -2.1;
	igrf11.gCoefficents[10][11] = 0.8;
	igrf11.hCoefficents[10][11] = -1.9;
	igrf11.gCoefficents[11][11] = 3.8;
	igrf11.hCoefficents[11][11] = -1.8;
	igrf11.gCoefficents[0][12] = -2.1;
	igrf11.gCoefficents[1][12] = -0.2;
	igrf11.hCoefficents[1][12] = -0.8;
	igrf11.gCoefficents[2][12] = 0.3;
	igrf11.hCoefficents[2][12] = 0.3;
	igrf11.gCoefficents[3][12] = 1.0;
	igrf11.hCoefficents[3][12] = 2.2;
	igrf11.gCoefficents[4][12] = -0.7;
	igrf11.hCoefficents[4][12] = -2.5;
	igrf11.gCoefficents[5][12] = 0.9;
	igrf11.hCoefficents[5][12] = 0.5;
	igrf11.gCoefficents[6][12] = -0.1;
	igrf11.hCoefficents[6][12] = 0.6;
	igrf11.gCoefficents[7][12] = 0.5;
	igrf11.hCoefficents[7][12] = 0.0;
	igrf11.gCoefficents[8][12] = -0.4;
	igrf11.hCoefficents[8][12] = 0.1;
	igrf11.gCoefficents[9][12] = -0.4;
	igrf11.hCoefficents[9][12] = 0.3;
	igrf11.gCoefficents[10][12] = 0.2;
	igrf11.hCoefficents[10][12] = -0.9;
	igrf11.gCoefficents[11][12] = -0.8;
	igrf11.hCoefficents[11][12] = -0.2;
	igrf11.gCoefficents[12][12] = 0.0;
	igrf11.hCoefficents[12][12] = 0.8;
	igrf11.gCoefficents[0][13] = -0.2;
	igrf11.gCoefficents[1][13] = -0.9;
	igrf11.hCoefficents[1][13] = -0.8;
	igrf11.gCoefficents[2][13] = 0.3;
	igrf11.hCoefficents[2][13] = 0.3;
	igrf11.gCoefficents[3][13] = 0.4;
	igrf11.hCoefficents[3][13] = 1.7;
	igrf11.gCoefficents[4][13] = -0.4;
	igrf11.hCoefficents[4][13] = -0.6;
	igrf11.gCoefficents[5][13] = 1.1;
	igrf11.hCoefficents[5][13] = -1.2;
	igrf11.gCoefficents[6][13] = -0.3;
	igrf11.hCoefficents[6][13] = -0.1;
	igrf11.gCoefficents[7][13] = 0.8;
	igrf11.hCoefficents[7][13] = 0.5;
	igrf11.gCoefficents[8][13] = -0.2;
	igrf11.hCoefficents[8][13] = 0.1;
	igrf11.gCoefficents[9][13] = 0.4;
	igrf11.hCoefficents[9][13] = 0.5;
	igrf11.gCoefficents[10][13] = 0.0;
	igrf11.hCoefficents[10][13] = 0.4;
	igrf11.gCoefficents[11][13] = 0.4;
	igrf11.hCoefficents[11][13] = -0.2;
	igrf11.gCoefficents[12][13] = -0.3;
	igrf11.hCoefficents[12][13] = -0.5;
	igrf11.gCoefficents[13][13] = -0.3;
	igrf11.hCoefficents[13][13] = -0.8;

	igrf11.gSV[0][1] = 11.4;
	igrf11.gSV[1][1] = 16.7;
	igrf11.hSV[1][1] = -28.8;
	igrf11.gSV[0][2] = -11.3;
	igrf11.gSV[1][2] = -3.9;
	igrf11.hSV[1][2] = -23.0;
	igrf11.gSV[2][2] = 2.7;
	igrf11.hSV[2][2] = -12.9;
	igrf11.gSV[0][3] = 1.3;
	igrf11.gSV[1][3] = -3.9;
	igrf11.hSV[1][3] = 8.6;
	igrf11.gSV[2][3] = -2.9;
	igrf11.hSV[2][3] = -2.9;
	igrf11.gSV[3][3] = -8.1;
	igrf11.hSV[3][3] = -2.1;
	igrf11.gSV[0][4] = -1.4;
	igrf11.gSV[1][4] = 2.0;
	igrf11.hSV[1][4] = 0.4;
	igrf11.gSV[2][4] = -8.9;
	igrf11.hSV[2][4] = 3.2;
	igrf11.gSV[3][4] = 4.4;
	igrf11.hSV[3][4] = 3.6;
	igrf11.gSV[4][4] = -2.3;
	igrf11.hSV[4][4] = -0.8;
	igrf11.gSV[0][5] = -0.5;
	igrf11.gSV[1][5] = 0.5;
	igrf11.hSV[1][5] = 0.5;
	igrf11.gSV[2][5] = -1.5;
	igrf11.hSV[2][5] = 1.5;
	igrf11.gSV[3][5] = -0.7;
	igrf11.hSV[3][5] = 0.9;
	igrf11.gSV[4][5] = 1.3;
	igrf11.hSV[4][5] = 3.7;
	igrf11.gSV[5][5] = 1.4;
	igrf11.hSV[5][5] = -0.6;
	igrf11.gSV[0][6] = -0.3;
	igrf11.gSV[1][6] = -0.3;
	igrf11.hSV[1][6] = -0.1;
	igrf11.gSV[2][6] = -0.3;
	igrf11.hSV[2][6] = -2.1;
	igrf11.gSV[3][6] = 1.9;
	igrf11.hSV[3][6] = -0.4;
	igrf11.gSV[4][6] = -1.6;
	igrf11.hSV[4][6] = -0.5;
	igrf11.gSV[5][6] = -0.2;
	igrf11.hSV[5][6] = 0.8;
	igrf11.gSV[6][6] = 1.8;
	igrf11.hSV[6][6] = 0.5;
	igrf11.gSV[0][7] = 0.2;
	igrf11.gSV[1][7] = -0.1;
	igrf11.hSV[1][7] = 0.6;
	igrf11.gSV[2][7] = -0.6;
	igrf11.hSV[2][7] = 0.3;
	igrf11.gSV[3][7] = 1.4;
	igrf11.hSV[3][7] = -0.2;
	igrf11.gSV[4][7] = 0.3;
	igrf11.hSV[4][7] = -0.1;
	igrf11.gSV[5][7] = 0.1;
	igrf11.hSV[5][7] = -0.8;
	igrf11.gSV[6][7] = -0.8;
	igrf11.hSV[6][7] = -0.3;
	igrf11.gSV[7][7] = 0.4;
	igrf11.hSV[7][7] = 0.2;
	igrf11.gSV[0][8] = -0.1;
	igrf11.gSV[1][8] = 0.1;
	igrf11.hSV[1][8] = 0.0;
	igrf11.gSV[2][8] = -0.5;
	igrf11.hSV[2][8] = 0.2;
	igrf11.gSV[3][8] = 0.3;
	igrf11.hSV[3][8] = 0.5;
	igrf11.gSV[4][8] = -0.3;
	igrf11.hSV[4][8] = 0.4;
	igrf11.gSV[5][8] = 0.3;
	igrf11.hSV[5][8] = 0.1;
	igrf11.gSV[6][8] = 0.2;
	igrf11.hSV[6][8] = -0.1;
	igrf11.gSV[7][8] = -0.5;
	igrf11.hSV[7][8] = 0.4;
	igrf11.gSV[8][8] = 0.2;
	igrf11.hSV[8][8] = 0.4;

	return igrf11;
}

long long factorial(unsigned int x)
{
	long long result = 1L;
	for(unsigned int i = 2; i <= x; i++)
		result *= i;
	return result;
}

long double legendreCos(unsigned int degree, unsigned int order, long double theta)
{
	if(theta != legendreTheta)//if using a new theta value
	{
		for(int i = 0; i <= MAX_LEGENDRE_ORDER; i++)
			for(int j = 0; j <= MAX_LEGENDRE_DEGREE; j++)
				legendreBoolean[i][j] = 0;//reset each boolean flag
		legendreTheta = theta;//and change the theta value
	}
	
	if(legendreBoolean[order][degree])
		return legendreValue[order][degree];
	
	if(degree == 0 && order == 0)//Base case P^0_0 = 1
	{
		legendreValue[0][0] = 1;
		legendreBoolean[0][0] = 1;
		return 1;
	}
		
	if(degree == 1 && order == 0)//Base case P^0_1 = x
	{
		legendreValue[0][1] = cos(theta);
		legendreBoolean[0][1] = 1;
		return cos(theta);
	}
	
	if(order > degree)
	{
		legendreValue[order][degree] = 0;
		legendreBoolean[order][degree] = 1;
		return 0;//if |m| > l then P^m_l = 0; here m and l are positive
	}
	
	if(order > 0)
	{
		if(cos(theta) == 1 || cos(theta) == -1)//P(-1) or P(1) is non zero if m == 0; m != 0;
		{
			legendreValue[order][degree] = 0;
			legendreBoolean[order][degree] = 1;
			return 0;
		}
		
		legendreValue[order][degree] = ((degree - order + 1) * cos(theta) * legendreCos(degree, order - 1, theta) - (degree + order - 1) * legendreCos(degree - 1, order - 1, theta)) / sin(theta);//sqrt(1 - x^2) * P^m+1_l(x) = (l - m) * x * P^m_l(x) - (l + m) * P^m_l-1(x)
		legendreBoolean[order][degree] = 1;
		return legendreValue[order][degree];
	}
	
	legendreValue[order][degree] = ((2 * degree - 1) * cos(theta) * legendreCos(degree - 1, order, theta) - (degree + order - 1) * legendreCos(degree - 2, order, theta))/(degree - order);//(l - m) P^m_l+1(x) = (2l + 1) * x * P^m_l(x) - (l + m) * P^m_l-1(x)
	legendreBoolean[order][degree] = 1;
	return legendreValue[order][degree];
}

long double schmidtLegendreCos(unsigned int degree, unsigned int order, long double theta)
{
	if(order == 0)
		return legendreCos(degree, 0, theta);
	return (1 - 2 * (order % 2)) * sqrt(2 * factorial(degree - order) / factorial(degree + order)) * legendreCos(degree, order, theta);
}

long double scalarPotential(IGRF model, long double radius, long double theta, long double phi, double year)
{
	long double sum = 0L;
	for(unsigned char n = 1; n <= model.maxDegree; n++)
	{
		long double subSum = 0L;
		for(unsigned char m = 0; m <= n; m++)
			subSum += ((model.gCoefficents[m][n] + model.gSV[m][n] * (year - model.baseYear)) * cos(m * phi) + (model.hCoefficents[m][n] + model.hSV[m][n] * (year - model.baseYear)) * sin(m * phi)) * schmidtLegendreCos(n, m, theta);
		subSum *= pow(EARTH_RADIUS / radius, n + 1);
		sum += subSum;
	}
	return EARTH_RADIUS * sum;
}

void calculateMagneticField(IGRF model, long double altitude, long double latitude, long double longetude, double time, double results[])
{
	long double radius = EARTH_RADIUS + altitude;
	long double theta = (90 - atan((1 - 1 / INVERSE_FLATTENING) * (1 - 1 / INVERSE_FLATTENING) * tan(latitude))) * acos(- 1.0L) / 180.0L;
	long double phi = longetude * acos(- 1.0L) / 180.0L;
	double year = time;
	
	/*long*/ double x = (scalarPotential(model, radius, theta + HALF_DELTA_THETA, phi, year) - scalarPotential(model, radius, theta - HALF_DELTA_THETA, phi, year)) / (2 * HALF_DELTA_THETA * radius);
	/*long*/ double y = (scalarPotential(model, radius, theta, phi + HALF_DELTA_PHI, year) - scalarPotential(model, radius, theta, phi - HALF_DELTA_PHI, year)) / (2 * HALF_DELTA_PHI * radius * sin(theta));
	/*long*/ double z = (scalarPotential(model, radius + HALF_DELTA_RADIUS, theta, phi, year) - scalarPotential(model, radius - HALF_DELTA_RADIUS, theta, phi, year)) / (2 * HALF_DELTA_RADIUS);
	
	results[0] = sqrt(x * x + y * y + z * z);
	results[1] = sqrt(x * x + y * y);
	results[2] = atan2(z, sqrt(x * x + y * y));
	results[3] = atan2(y, x);
}