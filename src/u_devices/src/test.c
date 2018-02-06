#include "marsboard.h"

int8_t adc1,adc2,adc3;

int main(void)
{
while(1)
{

if(digitalRead(START)==0)
{

	digitalWrite(LED_RED,1);
	adc1=analogRead(0);

	printf("adc1=%d\r\n",adc1);
	adc2=analogRead(1);

	printf("adc2=%d\r\n",adc2);
	adc3=analogRead(2);

	printf("adc3=%d\r\n",adc3);
}
else
    digitalWrite(LED_RED,0);
if(digitalRead(JOY_KEY)==0)
{
printf("OK");
    digitalWrite(LED_GREEN,1);
}
else
    digitalWrite(LED_GREEN,0);
}
    return 0;
}
