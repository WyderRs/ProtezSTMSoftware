/*
 * StarVar.h
 *
 *  Created on: Jun 19, 2026
 *      Author: makar
 */

#ifndef INC_STARVAR_H_
#define INC_STARVAR_H_


float PI           = 3.1415926;

/* p0 */
float p0x          = 0.0;          /* SET */
float p0y          = 0.0;          /* SET */
/* p1 */
// float p1x      = 40.36;
// float p1y      = 12.52;
/* p2 */
// float p2x          = 82.01;        /* SET */
// float p2y          = 15.39;        /* SET */

/* lines (x-y) */
float lx_p0p1      = 40.00;        /* SET */
float ly_p0p1      = 12.00;        /* SET */
float lx_p1p2      = 45.00;        /* SET */
float ly_p1p2      = 0.0;          /* SET */
/* lines_hp (module) */
float l_p0p1       = 0.0;          /* calc*/
float l_p1p2       = 45.00;        /* SET */
float l_p0p10      = 42.0;         /* SET */       /* PROC F */
float l_p2p3       = 74.7909;      /* SET */       /* Q0 */
float l_p1p3       = 40.00;        /* SET */
float l_p4p11      = 30.0;         /* SET */
float l_p3p4       = 59.26;        /* SET */       /* Q1 */
float l_p3p11      = 60.00;        /* SET */
float l_p0p3       = 0.0;          /* calc */
float l_p0p11      = 0.0;          /* calc */
float l_p8p10      = 24.0;         /* SET */       /* MIDLLE F */
float l_p4p9       = 30.0;         /* SET */
float l_p9p10      = 0.0;          /* calc */
float l_p9p11      = 0.0;          /* calc */
float l_p5p9       = 32.0;         /* calc */
float l_p4p5       = 30.91;        /* SET */       /* Q2 */
float l_p4p10      = 0.0;          /* calc */
float l_p10p11     = 0.0;          /* calc */
float l_p8p9       = 0.0;          /* calc */
float l_p5p8       = 0.0;          /* calc */
float l_p7p8       = 0.0;          /* calc */
float l_p6p8       = 22.0;         /* SET */       /* DIST F */
float l_p5p7       = 30.0;         /* SET */
float l_p6p7       = 0.0;          /* calc */


/* Offset Q[0, 1, 2] */
float Q_Ofst[3] = {0, };
/* angles */
//float fi_0         = (123.02 - 90.0) / 180.0 * PI;





#endif /* INC_STARVAR_H_ */
