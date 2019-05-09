#include"delta.h"

// trigonometric constants
const double sqrt3 = sqrt(3.0);
const double sin120 = sqrt3/2.0;   
const double cos120 = -0.5;        
const double tan60 = sqrt3;
const double sin30 = 0.5;
const double tan30 = 1/sqrt3;

// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
int delta::forward(double theta1, double theta2, double theta3,
			     double *x0, double *y0, double *z0)
{
	double t = (f-e)*tan30/2;
	double dtr = pi/(double)180.0;

	theta1 *= dtr;
	theta2 *= dtr;
	theta3 *= dtr;

	double y1 = -(t + rf*cos(theta1));
	double z1 = -rf*sin(theta1);

	double y2 = (t + rf*cos(theta2))*sin30;
	double x2 = y2*tan60;
	double z2 = -rf*sin(theta2);

	double y3 = (t + rf*cos(theta3))*sin30;
	double x3 = -y3*tan60;
	double z3 = -rf*sin(theta3);

	double dnm = (y2-y1)*x3-(y3-y1)*x2;

	double w1 = y1*y1 + z1*z1;
	double w2 = x2*x2 + y2*y2 + z2*z2;
	double w3 = x3*x3 + y3*y3 + z3*z3;

	// x = (a1*z + b1)/dnm
	double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
	double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

	// y = (a2*z + b2)/dnm;
	double a2 = -(z2-z1)*x3+(z3-z1)*x2;
	double b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

	// a*z^2 + b*z + c = 0
	double a = a1*a1 + a2*a2 + dnm*dnm;
	double b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
	double c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);

	// discriminant
	double d = b*b - (double)4.0*a*c;
	if (d < 0) return -1; // non-existing point

	*z0 = -(double)0.5*(b+sqrt(d))/a;
	*x0 = (a1*(*z0) + b1)/dnm;
	*y0 = (a2*(*z0) + b2)/dnm;
	return 0;
}

// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
int delta::calcAngleYZ(double x0, double y0, double z0, double *theta) {
	double y1 = -0.5 * tan30 * f; // f/2 * tg 30
	y0 -= 0.5 * tan30 * e;    // shift center to edge
	// z = a + b*y
	double a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
	double b = (y1-y0)/z0;
	// discriminant
	double d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
	if (d < 0) return -1; // non-existing point
	double yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
	double zj = a + b*yj;
	*theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
	return 0;
}

// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=non-existing position
int delta::inverse(double x0, double y0, double z0, double *theta1, double *theta2, double *theta3) {
	*theta1 = 0;
	*theta2 = 0;
	*theta3 = 0;
	int status = calcAngleYZ(x0, y0, z0, theta1);
	if (status == 0) status = calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
	if (status == 0) status = calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
	return status;
}

float delta::puls2rad(uint16_t pulse, int max_enc ){
	return (float)(pulse)/(float)(max_enc)*pi*2;
}

float delta::deg2rad(float angle){
	return (fmod(angle,360.0))/180.0*pi;
}

float delta::rad2deg(float rad){
	return fmod(rad/pi*180.0,360.0);
}

float delta::puls2ang(uint16_t pulse, int max_enc){
	return rad2deg(puls2rad(pulse, max_enc));
}


void delta::set_deg(double d1, double d2, double d3,
		    double *dac1, double *dac2, double *dac3 ){

	if(reverseM1) d1 = -d1;
	if(reverseM2) d2 = -d2;
	if(reverseM3) d3 = -d3;

	*dac1 = c1.calculate(deg2rad(d1), enc1);
	*dac2 = c2.calculate(deg2rad(d2), enc2);
	*dac3 = c3.calculate(deg2rad(d3), enc3);

	if(reverseM1) *dac1 = -(*dac1);
	if(reverseM2) *dac2 = -(*dac2);
	if(reverseM3) *dac3 = -(*dac3);

	tgt_d1 = d1;
	tgt_d2 = d2;
	tgt_d3 = d3;
}

void delta::set_pos(double x, double y, double z, 
		    double *dac1, double *dac2, double *dac3 ){
	double d1,d2,d3;
	z += 132.00; //?? empricaly found, makes motor angles = 0;
	inverse(x,y,z,&d1,&d2,&d3);

	if(reverseM1) d1 = -d1;
	if(reverseM2) d2 = -d2;
	if(reverseM3) d3 = -d3;

	*dac1 = c1.calculate(deg2rad(d1), enc1);
	*dac2 = c2.calculate(deg2rad(d2), enc2);
	*dac3 = c3.calculate(deg2rad(d3), enc3);

	if(reverseM1) *dac1 = -(*dac1);
	if(reverseM2) *dac2 = -(*dac2);
	if(reverseM3) *dac3 = -(*dac3);

	tgt_d1 = d1;
	tgt_d2 = d2;
	tgt_d3 = d3;
}

void delta::get_pos(double *x, double *y, double *z){
	forward(rad2deg(enc1),
		rad2deg(enc2),
		rad2deg(enc3),
		x, y, z);
}

void delta::get_deg(double *d1, double *d2, double *d3){
	*d1 = rad2deg(enc1);
	*d2 = rad2deg(enc2);
	*d3 = rad2deg(enc3);
}
void delta::get_tgt_deg(double *d1, double *d2, double *d3){
	*d1 = tgt_d1;
	*d2 = tgt_d2;
	*d3 = tgt_d3;
}

void delta::reset(){
	c1.reset();
	c2.reset();
	c3.reset();

	enc1 = 0;
	enc2 = 0;
	enc3 = 0;

	tgt_d1 = 0;
	tgt_d2 = 0;
	tgt_d3 = 0;
}

delta::delta(double e, double f, double re, double rf,
		  double ENC1, double ENC2, double ENC3,
		  bool reverseM1, bool reverseM2, bool reverseM3,
		  controller &c1, controller &c2, controller &c3)
	:e(e)
	,f(f)
	,re(re)
	,rf(rf)
	,ENC1(ENC1)
	,ENC2(ENC2)
	,ENC3(ENC3)
	,reverseM1(reverseM1)
	,reverseM2(reverseM2)
	,reverseM3(reverseM3)
	,c1(c1)
	,c2(c2)
	,c3(c3)
{
	reset();
}

delta::~delta(){
}

void delta::set_mtr_enc(int e1, int e2, int e3){

	if(reverseM1) enc1 = puls2rad(ENC1 - e1, ENC1);
	else enc1 = puls2rad(e1, ENC1);

	if(reverseM2) enc2 = puls2rad(ENC2 - e2, ENC2);
	else enc2 = puls2rad(e2, ENC2);

	if(reverseM3) enc3 = puls2rad(ENC3 - e3, ENC3);
	else enc3 = puls2rad(e3, ENC3);
}

