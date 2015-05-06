/*Controlador de Motor a Pasos Unipolar, modelo 28BYJ-48.
 * 5 Hilos, con código de colores. Conexión con el MSP430G2553
 * a través del puerto 2. Pines:
 * P2.0 -> Azul
 * P2.1 -> Rosa
 * P2.2 -> Amarillo
 * P2.3 -> Naranja
 * Rojo a 5V o 3.3 de la placa Launchpad
 * Conexión UART  9600 baudios, 8bits,sin paridad,1 stop bit
 * */

#include <msp430.h>
unsigned int iStep=0;	//Índice de paso
unsigned char cR='\0';	//Caracter leído
unsigned int iR=0;		//Índice del caracter Leído
unsigned char stepsH[8]={0x01,0x03,0x02,0x06,0x04,0x0c,0x08,0x09};	// Pasos sentido horario
unsigned char stepsA[8]={0x09,0x08,0x0c,0x04,0x06,0x02,0x03,0x01}; // Pasos sentido antihorario
unsigned char *steps;	//Apuntador al arreglo de pasos deseado
const unsigned int vel[10]={1000,2000,3000,4000,5000,6000,7000,8000,9000,900}; //10 Velocidades
char uart_getc();	//	Obtener un caracter
void ejecutarComando(unsigned char comando[]);

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  if (CALBC1_1MHZ==0xFF){					// If calibration constant erased
    while(1);                               // do not load, trap CPU!!
  }
  /*Configurar puertos*/
  P2DIR = 0x0F;			//P2 como salida en sus 4 bajos
  P2OUT = 0;			//Saco un 0 por el puerto 2
  P1DIR = BIT6;			//P1.6 = PWM
  P1SEL = BIT1 + BIT2 + BIT6 ;              // P1.1 = RXD, P1.2=TX, P1.6=PWM
  P1SEL2 = BIT1 + BIT2 ;                    // P1.1 = RXD, P1.2=TXD

  /*Configurar timmer TA0*/
  TA0CTL = TASSEL_2+MC_1;	//Reloj interno, modo de cuenta a TACCR0
  TA0CCTL1 = OUTMOD_7;		//Salida Reset/Set para TA0.1

 /*Configurar reloj*/
  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
  DCOCTL = CALDCO_1MHZ;						// Set DCO

  /*Configurar uart*/
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 = 104;                            // 1MHz 9600
  UCA0BR1 = 0;                              // 1MHz 9600
  UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  unsigned char comando[2];		//Primer Caracter:Sentido de giro; Segundo Caracter: velocidad; Ejemplo: H2

  /*Configurar ADC10*/
  /*Para la lectura del ADC10 no es necesario utilizar el P1SEL ni P1SEL2 */
  ADC10CTL0|=ADC10ON+ADC10IE+MSC;			//Conversor encendido, interrupción habilitada, Modo Continuo;
  ADC10CTL1|=INCH_3+ADC10SSEL_3+CONSEQ_2;	//Canal 3 de entrada, fuente de reloj SMCLK
  ADC10AE0=BIT3;							//Habilitación del A3 como entrada analógica
  ADC10CTL0|=ENC+ADC10SC;					//Se captura una vez el adc10

  __bis_SR_register(GIE);      		 		// interrupts enabled

  while(1){
	  if(iR<2){

	  comando[iR-1]=uart_getc();

	  }
	  else{
		  ejecutarComando(comando);
		  iR=0;
	  }
  }
}

char uart_getc(void){
	cR='\0';
	IE2|=UCA0RXIE;
 	while(cR=='\0');
	return cR;

}

void ejecutarComando(unsigned char _comando[]){

	switch(_comando[0]){//Comando para el sentido del motor
	case 'A':	//Sentido Antihorario
		steps=stepsA;
		TACTL|=TAIE;		//Encendemos la interrupción de timer, limpiamos el TA0R
		break;
	case 'H':	//Sentido Horario
		steps=stepsH;
		TACTL|=TAIE;		//Encendemos la interrupción de timer, limpiamos el TA0R
		break;
	case 'S':	//Stop, detener el motor
		TA0CCR0=0;
		TACTL&=~TAIE;		//Limpiamos la interrupción del timer
		TACTL&=~TAIFG;	//Se limpia el flag de interrupción de timer
		iStep=0;
		break;
	}
	if(_comando[0]!='S'){
		if(_comando[1]>='0' && _comando[1]<='9'){	//Si el char corresponde a un número
			TACCR0=vel[_comando[1]-48];		//Le damos a TACCR0 la velocidad de ese número en el arreglo
		}
	}
}


#pragma vector=USCIAB0RX_VECTOR
__interrupt void recep_isr(void){
	cR=UCA0RXBUF;				// Echo back RXed character, confirm TX buffer is ready first
	while (!(IFG2&UCA0TXIFG));	// USCI_A0 TX buffer ready?
	UCA0TXBUF = cR;			// TX -> RXed character
	if(++iR==2){
	IE2&=~UCA0RXIE;
	}
	IFG2&=~UCA0RXIFG;
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void timer_isr (void){	//ISR de timer
	iStep=(iStep<8)?iStep:0;		//Si el índice de pasos es menor a 8, no se altera, si no, se vuelve a 0
	P2OUT=steps[iStep++];			//Se saca el paso en el puerto 2
	TACTL &= ~ TAIFG;				//Se limpia el flag de interrupción de timer
}
#pragma vector=ADC10_VECTOR
__interrupt void adc10_isr(void){//ISR del ADC10
	TA0CCR1 = ADC10MEM;
	ADC10CTL0&=~ADC10IFG;
}
