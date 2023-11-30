  /*! \file *********************************************************************
 *
 * \brief
 *      LIAISON SERIE (Transmission)
 *
 * - Ce code configure la communication série USART sur un microcontrôleur AVR avec une fréquence d'horloge de 16 MHz et un taux de bauds de 9600 bps. 
 *
 * - La fonction USART_Transmit envoie continuellement le caractère ASCII 'A' (65) via la communication série. 
 *
 * - Ce programme peut être utilisé comme base pour tester la transmission série sur un microcontrôleur ou pour des applications simples nécessitant l'émission constante d'un caractère sur la ligne série.
 *

 *

 * \author
 * - Name: Aïmane Zelloufi 
 *
 * - e-mail: aimane.zelloufi@etudiant.univ-reims.fr \n
 *
 * $Name: - RELEASE_1_0 $
 * $Revision: - 1.0 $
 * $RCSfile: - transmission.c,v $
 * $Date: - 30/11/2023 12:30:00 $  \n
 ******************************************************************************/





/*! \brief 
 * - La directive #define FOSC 16000000 fixe la fréquence du quartz du microcontrôleur à 16 MHz. 
 * - Cette information est utilisée dans le calcul du taux de bauds lors de la configuration de la communication série USART. 
 * - La constante FOSC assure une adaptation précise des paramètres de communication.
 */
#define FOSC 16000000  // Clock Speed

/*! \brief 
 * - La directive `#define BAUD 9600` détermine le taux de bauds de la communication série USART à 9600 bits par seconde. 
 * - Cette constante est utilisée pour calculer la valeur du registre UBRR0, qui permet de configurer la communication série à la vitesse souhaitée. 
 * - La définition de `BAUD` facilite le réglage du taux de bauds dans le code sans avoir à le modifier directement.
 */
#define BAUD 9600

/*! \brief 
 * - La directive #define MYUBRR FOSC / 16 / BAUD - 1 calcule la valeur nécessaire pour le registre UBRR0 afin de configurer la communication série USART avec un taux de bauds de 9600 bps, en utilisant la fréquence du quartz FOSC de 16 MHz. 
 * - Cette constante MYUBRR est employée pour initialiser la communication série avec la vitesse de transmission désirée.
 */
#define MYUBRR FOSC / 16 / BAUD - 1

/*! \brief 
 * - Ce programme principal initialise la communication série USART avec un taux de bauds spécifié (MYUBRR) et transmet continuellement le caractère ASCII 'A' (65 en décimal) via la fonction USART_Transmit. 
 */
int main(void) {

  USART_Init(MYUBRR);
  while (1)
    
  {
    USART_Transmit(65);
  }
}

/*! \brief 
 * - La fonction USART_Init configure la communication série USART sur un microcontrôleur AVR. 
 * - Elle ajuste la vitesse de communication en configurant les registres de baud rate high et low, active le transmetteur USART, et définit le format du cadre de communication avec 8 bits de données et 2 bits d'arrêt. 
 * - Cette configuration standard assure une communication série fiable.
 */
void USART_Init(unsigned int ubrr) {
  /*Set baud rate */
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  /*Enable receiver and transmitter */
  UCSR0B = (1 << TXEN0);
  /* Set frame format: 8data, 2stop bit */
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

/*! \brief 
 * - La fonction USART_Transmit transmet un octet de données sur la communication série USART d'un microcontrôleur AVR.
 * - La boucle while assure l'attente du tampon de transmission vide.
 * - Le placement de la donnée dans le tampon de données (UDR0) déclenche le début de la transmission.
 * - Cette fonction est essentielle pour garantir une transmission synchrone, évitant la perte de données en attendant que le tampon soit disponible.
 */
void USART_Transmit(unsigned char data) {
  /* Wait for empty transmit buffer */
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  /* Put data into buffer, sends the data */
  UDR0 = data;
}


