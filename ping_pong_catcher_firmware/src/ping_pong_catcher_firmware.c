/*
 * ping_pong.c: program for the ping pong ball catcher
 *
 */

#include <stdio.h>
#include "platform.h"
#include "xparameters.h"
#include "xiomodule.h" // add
#include <math.h>
#include <limits.h>

#define M_PI_2		1.57079632679489661923
#define M_PI		3.14159265358979323846
#define U10_MAX 1023
// solve tdoa
#define n 3
#define ax 0.
#define ay 0.
#define bx 0.
#define by 320.
#define cx 320.
#define cy 320.
#define dx 320.
#define dy 0.
#define SERVO_0_DEG 51
#define SERVO_90_DEG 103
//#define v (int)(250000 / 50000.00 + .5) // m/s  from Anders who's an expert in waves
#define v (100000 / 50000000.00f) // m/s  from Anders who's an expert in waves
#define INTR_ID0  (XPAR_IOMODULE_INTC_MAX_INTR_SIZE - 1 - XPAR_IOMODULE_0_SYSTEM_INTC_INTERRUPT_0_INTR)
#define INTR_ID1  (XPAR_IOMODULE_INTC_MAX_INTR_SIZE - 1 - XPAR_IOMODULE_0_SYSTEM_INTC_INTERRUPT_1_INTR)
#define intrDelay 2500000
volatile float x[3] = {0};
volatile u32 hash;
XIOModule gpi;
XIOModule gpo;

void isr(void *par);

void print(char *str);
// void solveTdoa(u32 at, u32 bt, u32 ct, u32 dt, s32* x);
void solveTdoaDbl(float at, float bt, float ct, float dt);
float map_value_linear_range(float val, float from1, float from2, float to1, float to2);
u8 uartReadByte();
u8 intrFired;
u32 intrDelayCounter;
int main()
{
    init_platform();
    hash = 0;

    XIOModule_Initialize(&gpi, XPAR_IOMODULE_0_DEVICE_ID);
    XIOModule_Start(&gpi);
    XIOModule_Initialize(&gpo, XPAR_IOMODULE_0_DEVICE_ID);
    XIOModule_Start(&gpo);
    XIOModule_DiscreteWrite(&gpo, 1, 0x0001); // reset timing measurements

    // setup interrupts
    XIOModule NfdsIOMdule;
    XIOModule_Initialize(&NfdsIOMdule, XPAR_IOMODULE_0_DEVICE_ID);
    microblaze_register_handler(XIOModule_DeviceInterruptHandler, XPAR_IOMODULE_0_DEVICE_ID);
    XIOModule_Start(&NfdsIOMdule);
    XIOModule_Connect(&NfdsIOMdule, INTR_ID1, (XInterruptHandler)isr, XPAR_IOMODULE_0_DEVICE_ID);
    XIOModule_Enable(&NfdsIOMdule, INTR_ID1);
    microblaze_enable_interrupts();
    XIOModule_DiscreteWrite(&gpo, 1, 0x0000); // enable timing measurements

    intrFired = 0;
    intrDelayCounter = 0;

    while(1)
    {
    	/*
    	XIOModule_DiscreteWrite(&gpo, 1, 0x0001); // reset timing measurements

        u32 at = XIOModule_DiscreteRead(&gpi, 1);
    	u32 bt = XIOModule_DiscreteRead(&gpi, 2);
    	u32 ct = XIOModule_DiscreteRead(&gpi, 3);
    	u32 dt = XIOModule_DiscreteRead(&gpi, 4);

    	u32 angle = (u32)(map_value_linear_range(M_PI, 0, M_PI, SERVO_0_DEG, SERVO_90_DEG) + .5);
    	XIOModule_DiscreteWrite(&gpo, 2, angle);
    	xil_printf("angleVal = %d\r\n", angle);
    	 XIOModule_DiscreteWrite(&gpo, 2, angle);
    	xil_printf("at = %d, bt = %d, ct = %d, dt = %d\r\n", at, bt, ct, dt);
    	XIOModule_DiscreteWrite(&gpo, 1, 0x0000); // enable timing measurements
    	solveTdoaDbl(at, bt, ct, dt);
    	*/
    	//u32 resX = (u32)(x[0] + .5);
    	//u32 resY = (u32)(x[1] + .5);
		//xil_printf("%d,%d,%d\r\n",hash, resX, resY);

    	if(uartReadByte() == 'r')
    	{
    		u32 resX = (u32)(x[0] + .5);
    		u32 resY = (u32)(x[1] + .5);
    		xil_printf("%d,%d,%d\r\n",hash, resX, resY);
    	}
    	if(intrFired == 1)
    	{
    		intrDelayCounter++;
    		if(intrDelayCounter > intrDelay)
    		{
    			XIOModule_DiscreteWrite(&gpo, 1, 0x0000); // enable timing measurements
    			intrFired = 0;
    			XIOModule_DiscreteWrite(&gpo, 2, (u32)(SERVO_0_DEG));
    		}
    	}

		//printf("x = %d", x[0]);

		// for (int i=0; i<25000000; i++){};
    }
    cleanup_platform();
    return 0;
}

void isr(void *par)
{
	if(intrFired == 1)
		return;
    XIOModule_DiscreteWrite(&gpo, 1, 0x0001); // reset timing measurements
    // read timings
    u32 at = XIOModule_DiscreteRead(&gpi, 1);
	u32 bt = XIOModule_DiscreteRead(&gpi, 4);
	u32 ct = XIOModule_DiscreteRead(&gpi, 2);
	u32 dt = XIOModule_DiscreteRead(&gpi, 3);
	xil_printf("at = %d, bt = %d, ct = %d, dt = %d\r\n", at, bt, ct, dt);

	solveTdoaDbl(at, bt, ct, dt); // set x to solution of tdoa
	float radFloat = atanf(x[1]/x[0]);
	if(radFloat < 0.0f)
	{
			xil_printf("Error angle at = %d\r\n", radFloat);
			radFloat = 0.0f;
	}
	u32 angle = (u32)(map_value_linear_range(radFloat, 0, M_PI_2, SERVO_0_DEG, SERVO_90_DEG) + .5);
	xil_printf("angleVal = %d\r\n", angle);
	u32 resX = (u32)(x[0] + .5);
	u32 resY = (u32)(x[1] + .5);
	xil_printf("%d,%d,%d\r\n",hash, resX, resY);
	XIOModule_DiscreteWrite(&gpo, 2, angle);
	hash++;
	intrFired = 1;
	intrDelayCounter = 0;
}

void solveTdoaDbl(float at, float bt, float ct, float dt)
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
			//print("Singular matrix\n");
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
	return (u8)XIomodule_In32(STDIN_BASEADDRESS + XUL_RX_OFFSET);
}

/* integer version working
void solveTdoa(u32 at, u32 bt, u32 ct, u32 dt, s32* x)
{
	int vSquare = v*v;
	int a[3][3];
	a[0][0] = ax - bx;
	a[0][1] = ay - by;
	a[0][2] = vSquare*(bt - at);
	a[1][0] = bx - cx;
	a[1][1] = by - cy;
	a[1][2] = vSquare*(ct - bt);
	a[2][0] = cx - dx;
	a[2][1] = cy - dy;
	a[2][2] = vSquare*(dt - ct);
	int b[3];
	b[0] = (ax*ax + ay*ay - bx*bx - by*by - vSquare*(at*at - bt*bt)) / 2;
	b[1] = (bx*bx + by*by - cx*cx - cy*cy - vSquare*(bt*bt - ct*ct)) / 2;
	b[2] = (cx*cx + cy*cy - dx*dx - dy*dy - vSquare*(ct*ct - dt*dt)) / 2;
	// gauss-elimination
	for (unsigned char j = 0; j < n - 1; j++)
	{
		// pivoting
		// find row with largest element in pivot row
		unsigned char largestPivotRow = j;
		int largestPivotElement = abs(a[j][j]);
		int testElement;
		for (unsigned char i = j + 1; i < n; i++)
		{
			testElement = abs(a[i][j]);
			if (testElement > largestPivotElement)
			{
				largestPivotElement = testElement;
				largestPivotRow = i;
			}
		}
		if (largestPivotElement < 1E-6)
		{
			//print("Singular matrix\n");
			return;
		}


		// swap largest with current row
		for (unsigned char i = 0; i < n; i++)
		{
			int tmp = a[j][i];
			a[j][i] = a[largestPivotRow][i];
			a[largestPivotRow][i] = tmp;
		}

		// also for right hand side
		int tmpB = b[j];
		b[j] = b[largestPivotRow];
		b[largestPivotRow] = tmpB;
		// elimination
		for (unsigned char i = j + 1; i < n; i++)
		{
			int mult = a[i][j] / a[j][j];
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
*/
