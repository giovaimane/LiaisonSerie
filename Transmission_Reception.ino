/*! \file *********************************************************************


* \brief
 *      LIASON SERIE (Transmission-Réception)
 *
 * - Ce programme en langage C pour microcontrôleur AVR configure la communication série USART avec un taux de bauds de 9600 bps.
 * 
 * - Il l'utilise des interruptions pour traiter les caractères reçus, les incrémente, puis les renvoie. 
 *
 * - La fonction principale envoie périodiquement le caractère 'A' via la communication série. Les constantes définies facilitent l'ajustement du taux de bauds. 
 *
 * - Ce code sert de base pour des applications nécessitant une communication asynchrone sur un microcontrôleur AVR.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>  // N'oubliez pas d'inclure cette bibliothèque pour utiliser _delay_ms


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
 * - La déclaration unsigned char data0 crée une variable data capable de stocker des entiers non signés de 8 bits. 
 * - Elle est utilisée pour stocker des données reçues via la communication série USART0. 
 * - La taille de la variable est limitée à des valeurs entre 0 et 255.
 */
unsigned char data0;


/*! \brief 
 * - La déclaration unsigned char data3 crée une variable data capable de stocker des entiers non signés de 8 bits. 
 * - Elle est utilisée pour stocker des données reçues via la communication série USART3. 
 * - La taille de la variable est limitée à des valeurs entre 0 et 255.
 */
unsigned char data3;

/*! \brief 
 * - La déclaration unsigned char flag0 = 0; crée une variable nommée flag de type entier non signé de 8 bits, initialement mise à 0. 
 * - Cette variable est souvent utilisée comme un indicateur (drapeau) dans les programmes pour signaler un état particulier. 
 * - Dans ce contexte, elle est employée pour indiquer si des données ont été reçues via la communication série USART0, avec la valeur 1 pour indiquer la réception de données et 0 sinon.
 */
unsigned char flag0 =0;

/*! \brief 
 * - La déclaration unsigned char flag3 = 0; crée une variable nommée flag de type entier non signé de 8 bits, initialement mise à 0. 
 * - Cette variable est souvent utilisée comme un indicateur (drapeau) dans les programmes pour signaler un état particulier. 
 * - Dans ce contexte, elle est employée pour indiquer si des données ont été reçues via la communication série USART3, avec la valeur 1 pour indiquer la réception de données et 0 sinon.
 */
unsigned char flag3 =0;

/*! \brief 
 * - La fonction USART0_Init initialise la communication série USART0 sur un microcontrôleur AVR. 
 * - Elle configure le registre de baud rate avec la valeur fournie, active le récepteur et le transmetteur USART0, active l'interruption de réception pour permettre l'utilisation d'interruptions lors de la réception de données, et fixe le format du cadre de communication avec 8 bits de données et 1 bit d'arrêt. 
 * - Cette configuration standard assure une communication asynchrone avec des caractéristiques spécifiques. 
 * - L'utilisation de la fonction USART0_Init simplifie la configuration de la communication série dans les programmes principaux en appelant simplement cette fonction.
 */
void USART0_Init(unsigned int ubrr) {
  /*Set baud rate */
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  /*Enable receiver and transmitter */
  UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
  /* Set frame format: 8data, 1stop bit */
  UCSR0C =  (3 << UCSZ00);
}

/*! \brief 
 * - La fonction USART3_Init initialise la communication série USART sur un microcontrôleur AVR. 
 * - Elle configure le registre de baud rate avec la valeur fournie, active le récepteur et le transmetteur USART3, active l'interruption de réception pour permettre l'utilisation d'interruptions lors de la réception de données, et fixe le format du cadre de communication avec 8 bits de données et 1 bit d'arrêt. 
 * - Cette configuration standard assure une communication asynchrone avec des caractéristiques spécifiques. 
 * - L'utilisation de la fonction USART3_Init simplifie la configuration de la communication série dans les programmes principaux en appelant simplement cette fonction.
 */
void USART3_Init(unsigned int ubrr) {
  /*Set baud rate */
  UBRR3H = (unsigned char)(ubrr >> 8);
  UBRR3L = (unsigned char)ubrr;
  /*Enable receiver and transmitter */
  UCSR3B = (1 << TXEN3) | (1 << RXEN3) | (1 << RXCIE3);
  /* Set frame format: 8data, 1stop bit */
  UCSR3C =  (3 << UCSZ30);
}

/*! \brief 
 * - La fonction USART0_Transmit transmet un octet de données sur la communication série USART0 d'un microcontrôleur AVR.
 * - La boucle while assure l'attente du tampon de transmission vide.
 * - Le placement de la donnée dans le tampon de données (UDR0) déclenche le début de la transmission.
 * - Cette fonction est essentielle pour garantir une transmission synchrone, évitant la perte de données en attendant que le tampon soit disponible.
 */
void USART0_Transmit(unsigned char data0) {
  /* Wait for empty transmit buffer */
  while (!(UCSR0A & (1 << UDRE0)))    ;
  /* Put data into buffer, sends the data */
  UDR0 = data0;
}

/*! \brief 
 * - La fonction USART3_Transmit transmet un octet de données sur la communication série USART3 d'un microcontrôleur AVR.
 * - La boucle while assure l'attente du tampon de transmission vide.
 * - Le placement de la donnée dans le tampon de données (UDR3) déclenche le début de la transmission.
 * - Cette fonction est essentielle pour garantir une transmission synchrone, évitant la perte de données en attendant que le tampon soit disponible.
 */
void USART3_Transmit(unsigned char data3) {
  /* Wait for empty transmit buffer */
  while (!(UCSR3A & (1 << UDRE3)))    ;
  /* Put data into buffer, sends the data */
  UDR3 = data3;
}

/*! \brief 
 * - La fonction USART0_Receive attend que le tampon de réception soit prêt en vérifiant le bit d'indicateur d'attente de tampon plein (RXC0), puis retourne les données reçues du registre de données de réception (UDR0). 
 * - Cette fonction facilite la récupération synchrone des données reçues sur la communication série USART0 sans risque de lire des données non prêtes.
 */
unsigned char USART0_Receive() {
  // Attente que le tampon de réception soit prêt
  while (!(UCSR0A & (1 << RXC0)));

  // Retourner les données reçues du registre de données et incrémenter le code ASCII
  return UDR0;
}


/*! \brief 
 * - La fonction USART3_Receive attend que le tampon de réception soit prêt en vérifiant le bit d'indicateur d'attente de tampon plein (RXC3), puis retourne les données reçues du registre de données de réception (UDR3). 
 * - Cette fonction facilite la récupération synchrone des données reçues sur la communication série USAR3 sans risque de lire des données non prêtes.
 */
unsigned char USART3_Receive() {
  // Attente que le tampon de réception soit prêt
  while (!(UCSR3A & (1 << RXC3)));

  // Retourner les données reçues du registre de données et incrémenter le code ASCII
  return UDR3;
}


/*! \brief 
 * - Cette interruption de réception USART (ISR(USART0_RX_vect)) est une fonction à exécuter lorsque des données sont reçues sur la communication série. 
 * - Elle récupère la donnée reçue (data0=UDR0;) dans la variable data et active le drapeau (flag0=1;). 
 * - Cette approche est couramment utilisée pour indiquer dans le programme principal qu'une nouvelle donnée est disponible et peut être traitée. 
 * - La variable flag pourrait être utilisée comme un indicateur pour déclencher des actions dans le code principal en réponse à la réception de données.
 */
ISR(USART0_RX_vect) {
  // Fonction à exécuter lors de l'interruption de réception USART0
  //data0=UDR0;
  flag0=1;
  
  
}

/*! \brief 
 * - Cette interruption de réception USART (ISR(USART3_RX_vect)) est une fonction à exécuter lorsque des données sont reçues sur la communication série. 
 * - Elle récupère la donnée reçue (data3=UDR3;) dans la variable data et active le drapeau (flag3=1;). 
 * - Cette approche est couramment utilisée pour indiquer dans le programme principal qu'une nouvelle donnée est disponible et peut être traitée. 
 * - La variable flag pourrait être utilisée comme un indicateur pour déclencher des actions dans le code principal en réponse à la réception de données.
 */
ISR(USART3_RX_vect) {
  // Fonction à exécuter lors de l'interruption de réception USART3
  //data3=UDR3;
  flag3=1;
  
  
}

/*! \brief 
 * - Le programme principal initialise les interruptions globales et la communication série USART0 et USART3 avec un certain taux de bauds. 
 * - Ensuite, dans la boucle infinie, il vérifie périodiquement si des données ont été reçues (indiqué par le drapeau flag0 et flag3). 
 * - Si c'est le cas, il transmet ces données incrémentées de 1 via la communication série. 
 * - Le délai de 1 milliseconde (_delay_ms(1)) assure une temporisation courte entre les itérations de la boucle principale. 
 * - Cette structure de programme permet une réactivité aux interruptions, adaptée à la communication asynchrone.
 */
int main(void) {
 
  USART0_Init(MYUBRR);
  USART3_Init(MYUBRR);
  DDRB=0b10000000;
  sei();  // Activer les interruptions globales
  while (1) {

    if(flag3){
      USART0_Transmit(UDR3);
      flag3=0;
      PORTB = 0b10000000;
    }

        if(flag0){
      USART3_Transmit(UDR0);
      flag0=0;
      PORTB = 0b10000000;
    }

   _delay_ms(1);

    
  }
}
