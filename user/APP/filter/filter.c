#include "filter.h"
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
float y1,y2,y3,y4;
	
double NUM[5] = {
  0.0001298496353869,0.0005193985415477,0.0007790978123215,0.0005193985415477,
  0.0001298496353869
};
double DEN[5] = {
                   1,   -3.607896169129,    4.979470803751,   -3.110763682983,
     0.7415201473558
};
double NUM1[5] = {
  0.004824343357716,    0.019297373430865   ,0.028946060146297   ,0.019297373430865,
  0.004824343357716
};
double DEN1[5] = {
                   1,   -2.369513007182038,    2.313988414415880,   -1.054665405878568,
     0.187379492368185
};

double NUM_y_speed[3] = {-1.447215728956508,	4.990954366211414,		-3.538030494370182};//{0,		3.218587553256984,		-5.495268302825553,		2.279229873523972};//{4.000000000000000 , -3.990000000000000};//{-7.529583817442483,	18.496805951311064,		-10.957885786288221};//
double DEN_y_speed[3] = {1.000000000000000,		-1.000000000009464,		0.000000000009461};//{1,		-1.563890211615693,		0.574357183284079,		-0.010466971668386};//{1,-1};//{1.000000000000000  ,-1.000000000000029   ,0.000000000000000};//

void Chebyshev50HzLPF(Filter_t *F)
{
	int i;
	for(i=4; i>0; i--)
	{
		F->ybuf[i] = F->ybuf[i-1]; 
		F->xbuf[i] = F->xbuf[i-1];
	}
	F->xbuf[0] = F->raw_value;
	F->ybuf[0] = NUM[0] * F->xbuf[0];
	for(i=1;i<5;i++)
	{
		F->ybuf[0] = F->ybuf[0] + NUM[i] * F->xbuf[i] - DEN[i] * F->ybuf[i];
	}
	F->filtered_value = F->ybuf[0];
}
void Chebyshev100HzLPF(Filter_t *F)
{
	int i;
	for(i=4; i>0; i--)
	{
		F->ybuf[i] = F->ybuf[i-1]; 
		F->xbuf[i] = F->xbuf[i-1];
	}
	F->xbuf[0] = F->raw_value;
	F->ybuf[0] = NUM1[0] * F->xbuf[0];
	for(i=1;i<5;i++)
	{
		F->ybuf[0] = F->ybuf[0] + NUM1[i] * F->xbuf[i] - DEN1[i] * F->ybuf[i];
	}
	F->filtered_value = F->ybuf[0];
}

void Corrector_Yaw_Speed(Filter_t *F)
{
	int i;
	for(i=(sizeof(NUM_y_speed)/8); i>0; i--)
	{
		F->ybuf[i] = F->ybuf[i-1]; 
		F->xbuf[i] = F->xbuf[i-1];
	}
	F->xbuf[0] = F->raw_value;
	F->ybuf[0] = NUM_y_speed[0] * F->xbuf[0];
	for(i=1;i<(sizeof(NUM_y_speed)/8);i++)
	{
		F->ybuf[0] = F->ybuf[0] + NUM_y_speed[i] * F->xbuf[i] - DEN_y_speed[i] * F->ybuf[i];
	}
	LimitMax(F->ybuf[0],6);
	F->filtered_value = F->ybuf[0];
}


