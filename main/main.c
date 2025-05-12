#include <stdio.h>
#include "CS816D.h"
#include "ASTIdef.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

U2 u2_x, u2_y;
U1 u1_Gesture;

void app_main(void)
{
	CS816D_Init();
	while (1)
	{
		U1 u1_FingerIndex ;
		u1_FingerIndex = u1_CS816D_Read_Touch(&u2_x, &u2_y, &u1_Gesture);
		if (u1_FingerIndex)
		{
			printf("Finger Index: %d, X: %d, Y: %d, Gesture: %d\n", u1_FingerIndex, u2_x, u2_y, u1_Gesture);
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}