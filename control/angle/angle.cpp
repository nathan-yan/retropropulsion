#include <Arduino.h>
#include "../../util/utils.h"

static double bottomlink_startAngle = 103.3;
static double bottomlink_l1 = 52.5;
static double bottomlink_l2 = 10;
// static double bottomlink_x0 = bottomlink_l1 * cos(DegreeToR(bottomlink_startAngle)) + 30;  // 30
static double bottomlink_x0 = 12.5;
// static double bottomlink_y0 = bottomlink_l1 * sin(DegreeToR(bottomlink_startAngle)) - bottomlink_l2 - 1;
static double bottomlink_y0 = bottomlink_l1 * sin(DegreeToR(bottomlink_startAngle)) - bottomlink_l2 + 1;

static double toplink_startAngle = 101.5                                                                                                      ;
static double toplink_l1 = 53.5;
static double toplink_l2 = 10;
//static double toplink_x0 = toplink_l1 * cos(DegreeToR(toplink_startAngle)) + 30;
static double toplink_x0 = 13;
static double toplink_y0 = toplink_l1 * sin(DegreeToR(toplink_startAngle)) + toplink_l2 - 2;


// 3 bar problem
// 1. Long arm length is l1.  Fixed end is at (0, 0) and moveing end starts at (l1*cos(startAngle), l1*Sin(startAngle))
// 2. short arm length is l2.  Fixed end is at (x0, y0) moving end starts at (x0, y0+l2)
double linkage(double rocketAngle)
{
  double retVal = 0;
  double d2 = pow(bottomlink_x0 - bottomlink_l1 * cos(DegreeToR(bottomlink_startAngle)), 2)
              + pow(bottomlink_y0 + bottomlink_l2 - bottomlink_l1 * sin(DegreeToR(bottomlink_startAngle)), 2);

  double a1 = bottomlink_l1 * cos(DegreeToR(rocketAngle)) - bottomlink_x0;
  double b1 = bottomlink_l1 * sin(DegreeToR(rocketAngle)) - bottomlink_y0;
  double a12plusb12 = pow(a1, 2) + pow(b1, 2);
  double angle1 = atan(b1 / a1);
  double angle2 = acos((-a12plusb12 - pow(bottomlink_l2, 2) + d2) / (2 * bottomlink_l2 * sqrt(a12plusb12)));

  retVal = atan(b1 / a1) + acos((-a12plusb12 - pow(bottomlink_l2, 2) + d2) / (2 * bottomlink_l2 * sqrt(a12plusb12)));

  return RToDegree(retVal);
}

// 3 bar problem
// 1. Long arm length is l1.  Fixed end is at (0, 0) and moveing end starts at (l1*cos(startAngle), l1*Sin(startAngle))
// 2. short arm length is l2.  Fixed end is at (x0, y0) moving end starts at (x0, y0-l2)
double linkage2(double rocketAngle)
{
  double retVal = 0;
  double d2 = pow(toplink_x0 - toplink_l1 * cos(DegreeToR(toplink_startAngle)), 2)
              + pow(toplink_y0 - toplink_l2 - toplink_l1 * sin(DegreeToR(toplink_startAngle)), 2);

  double a1 = toplink_l1 * cos(DegreeToR(rocketAngle)) - toplink_x0;
  double b1 = toplink_l1 * sin(DegreeToR(rocketAngle)) - toplink_y0;
  double a12plusb12 = pow(a1, 2) + pow(b1, 2);
  double angle1 = atan(b1 / a1);
  double angle2 = acos((-a12plusb12 - pow(toplink_l2, 2) + d2) / (2. * toplink_l2 * sqrt(a12plusb12)));

  retVal = atan(b1 / a1) - acos((-a12plusb12 - pow(toplink_l2, 2) + d2) / (2. * toplink_l2 * sqrt(a12plusb12)));

  return RToDegree(retVal);
}