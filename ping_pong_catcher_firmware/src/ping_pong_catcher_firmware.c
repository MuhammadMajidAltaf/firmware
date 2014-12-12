/*
 * ping_pong.c: program for the ping pong ball catcher
 * Made by: Kim, Jorge and Anders
 * Develop as part of EMB1 a part of the master edu. in robot technology
 * Develop during easter 2014
 */

/************************************ Includes *******************************************/
// xilinx Microblaze
#include "platform.h"
#include "xparameters.h"
#include "xiomodule.h"
// standard C
#include <stdio.h>
#include <math.h>
#include <limits.h>

/************************************ Defines *******************************************/

//#define DEBUG 1

// solve tdoa constants
#define n 3
#define ax 0.0f
#define ay 0.0f
#define bx 0.0f
#define by 320.0f
#define cx 320.0f
#define cy 320.0f
#define dx 320.0f
#define dy 0.0f
#define v (250000.0f / 50000000.00f) // 250 m/s  from Anders who's an expert in waves

// constants for servo control
#define M_PI_2 1.57079632679489661923f
#define M_PI 3.14159265358979323846f
#define U10_MAX 1023
#define SERVO_0_DEG 48
#define SERVO_90_DEG 97

// constant for interrupts
#define INTR_ID0  (XPAR_IOMODULE_INTC_MAX_INTR_SIZE - 1 - XPAR_IOMODULE_0_SYSTEM_INTC_INTERRUPT_0_INTR)
#define INTR_ID1  (XPAR_IOMODULE_INTC_MAX_INTR_SIZE - 1 - XPAR_IOMODULE_0_SYSTEM_INTC_INTERRUPT_1_INTR)
#define intrDelay 1000000

/************************************ Functions declarations *******************************************/

/*
* ISR for timings measurements
* Postconditions:
* x contains the impact position
* Writes the angle to GPO2
*/
void isr(void *par);

/*
 * Prints string to UART
 */
void print(char *str);

/*
 * Solves the TDOA problem with floating point precision
 * Uses Gauss-elimination
 * Preconditions
 * at, bt, ct and dt contains measurements of difference in time measured in counts taken with 50Mhz
 */
void solveTdoaFlt(float at, float bt, float ct, float dt);

/*
 * Maps linear values to other span
 */
float map_value_linear_range(float val, float from1, float from2, float to1, float to2);

/*
 * Reads a byte on uart without blocking
 */
u8 uartReadByte();

/************************************ Variables *******************************************/

XIOModule gpio; // gpio struct
volatile float x[3] = {0}; // solution of TDOA [x, y, t]
volatile u32 hash; // no. of hits since start
volatile u8 intrFired; // bool that is one if ball has hit and are beeing handled
u32 intrDelayCounter; // helping variable used to make a non-blocking delay before arm goes to starting pos.

/************************************ Main program *******************************************/
int main()
{
	// setup
    init_platform();
    hash = 0;
    // GPIO setup
    XIOModule_Initialize(&gpio, XPAR_IOMODULE_0_DEVICE_ID);
    XIOModule_Start(&gpio);
    // Need to call CfgInitialize to use recive from UART
    // int XIOModule_CfgInitialize(XIOModule *InstancePtr, XIOModule_Config *Config, u32 EffectiveAddr);
    // note config and effective address arguments are not used
    XIOModule_CfgInitialize(&gpio, NULL, 1);

    // setup interrupts
    microblaze_register_handler(XIOModule_DeviceInterruptHandler, XPAR_IOMODULE_0_DEVICE_ID);
    XIOModule_Connect(&gpio, INTR_ID1, (XInterruptHandler)isr, XPAR_IOMODULE_0_DEVICE_ID);
    XIOModule_Enable(&gpio, INTR_ID1);
    microblaze_enable_interrupts();

    // reset timing measurements
    XIOModule_DiscreteWrite(&gpio, 1, 0x0001);
    XIOModule_DiscreteWrite(&gpio, 1, 0x0000); // enable timing measurements
    // prepare to run
    intrFired = 0;
    intrDelayCounter = 0;
    XIOModule_DiscreteWrite(&gpio, 2, (u32)(SERVO_90_DEG)); // move to start pos
    // main loop
    while(1)
    {
    	if(uartReadByte() == 'r')
    	{
    		u32 resX = (u32)(x[0] + .5);
    		u32 resY = (u32)(x[1] + .5);
    		xil_printf("%d,%d,%d\n",hash, resX, resY);
       	}

    	if(intrFired == 1)
    	{
    		// interrupt has fired
    		intrDelayCounter++;
    		if(intrDelayCounter > intrDelay)
    		{
    			// enough time since interupt
    			// move to start pos
    			XIOModule_DiscreteWrite(&gpio, 2, (u32)(SERVO_90_DEG));
    			intrFired = 0;
    			// enable timing measurements
    			XIOModule_DiscreteWrite(&gpio, 1, 0x0000);
    		}

    	}
    }
    cleanup_platform();
    return 0;
}
/************************************ Functions implementation *******************************************/
void isr(void *par)
{
	if(intrFired == 1)
		return;
    XIOModule_DiscreteWrite(&gpio, 1, 0x0001); // reset timing measurements
    // read timings
    u32 at = XIOModule_DiscreteRead(&gpio, 1);
	u32 bt = XIOModule_DiscreteRead(&gpio, 4);
	u32 ct = XIOModule_DiscreteRead(&gpio, 2);
	u32 dt = XIOModule_DiscreteRead(&gpio, 3);

	solveTdoaFlt(at, bt, ct, dt); // sets x to solution of tdoa
	// calc. and output angle
	float angleRad= atanf(x[1]/x[0]);
	if(angleRad< 0.0f)
	{
#ifdef DEBUG
		xil_printf("Error angle at = %d\r\n", angleRad);
#endif
			angleRad= 0.0f;
	}
	u32 angle = (u32)(map_value_linear_range(angleRad, 0, M_PI_2, SERVO_0_DEG, SERVO_90_DEG) + .5);
	XIOModule_DiscreteWrite(&gpio, 2, angle);

	hash++; // no.
	intrFired = 1;
	intrDelayCounter = 0;

#ifdef DEBUG
	//xil_printf("timeMes%d = [ta = %d;tb = %d;tc = %d;td = %d;]\r\n", hash, at, bt, ct, dt);
	xil_printf("time%d = [%d,%d,%d,%d];\r\n", hash, at, bt, ct, dt);
	//xil_printf("angleVal = %d\r\n", (u32)(angleRad*180/M_PI+.5)); // for debug
	//Debug output the values send to the computer program
	u32 resX = (u32)(x[0] + .5);
	u32 resY = (u32)(x[1] + .5);
	xil_printf("pos%d = [%d,%d];\r\n", hash, resX, resY);
#endif

}

void solveTdoaFlt(float at, float bt, float ct, float dt)
{
	float vSquare = v*v;
	float a[3][3];
	a[0][0] = ax - bx;
	a[0][1] = ay - by;
	a[0][2] = vSquare*(bt - at);
	a[1][0] = bx - cx;
	a[1][1] = by - cy;
	a[1][2] = vSquare*(ct - bt);
	a[2][0] = cx - dx;
	a[2][1] = cy - dy;
	a[2][2] = vSquare*(dt - ct);
	float b[3];
	b[0] = (ax*ax + ay*ay - bx*bx - by*by - vSquare*(at*at - bt*bt)) / 2;
	b[1] = (bx*bx + by*by - cx*cx - cy*cy - vSquare*(bt*bt - ct*ct)) / 2;
	b[2] = (cx*cx + cy*cy - dx*dx - dy*dy - vSquare*(ct*ct - dt*dt)) / 2;
	// gauss-elimination
	for (unsigned char j = 0; j < n - 1; j++)
	{
		// pivoting
		// find row with largest element in pivot row
		unsigned char largestPivotRow = j;
		float largestPivotElement = fabs(a[j][j]);
		float testElement;
		for (unsigned char i = j + 1; i < n; i++)
		{
			testElement =fabsf(a[i][j]);
			if (testElement > largestPivotElement)
			{
				largestPivotElement = testElement;
				largestPivotRow = i;
			}
		}
		if (largestPivotElement < 1E-6)
		{
#ifdef DEBUG
			print("Singular matrix\n");
#endif
			return;
		}

		// swap largest with current row
		for (unsigned char i = 0; i < n; i++)
		{
			float tmp = a[j][i];
			a[j][i] = a[largestPivotRow][i];
			a[largestPivotRow][i] = tmp;
		}

		// also for right hand side
		float tmpB = b[j];
		b[j] = b[largestPivotRow];
		b[largestPivotRow] = tmpB;
		// elimination
		for (unsigned char i = j + 1; i < n; i++)
		{
			float mult = a[i][j] / a[j][j];
			for (unsigned char c = j + 1; c < n; c++)
			{
				a[i][c] -= mult*a[j][c];
			}
			b[i] -= mult*b[j];
		}
	}
	// back substitution
	for (int i = n - 1; i >= 0; i--)
	{
		for (unsigned char j = i + 1; j < n; j++) // false first time
		{
			b[i] = b[i] - a[i][j] * x[j];
		}
		x[i] = b[i] / a[i][i];
	}
}

float map_value_linear_range(float val, float from1, float from2, float to1, float to2)
{
	return to1 + (val - from1) * (to2 - to1) / (from2 - from1);
}

u8 uartReadByte()
{
	if(XIOModule_IsReceiveEmpty(STDIN_BASEADDRESS))
		return 0;
	return (u8)XIomodule_In8(STDIN_BASEADDRESS + XUL_RX_OFFSET);
}
