
// PERIPHERALS
#define RCC_APB2ENR     (*((volatile unsigned long *) 0x40021018))
#define GPIO_A          (*((volatile unsigned long *) 0x40010800))
#define GPIO_B          (*((volatile unsigned long *) 0x40010C00))
#define GPIO_C          (*((volatile unsigned long *) 0x40011000))

// PORTS
#define PA 1
#define PB 2
#define PC 3

// DIRECTIONS
#define IN 0
#define OUT10 1
#define OUT2  2
#define OUT50 3

// INPUT OPTIONS
#define I_AN 0
#define I_F  1
#define I_PP 2

// OUTPUT OPTIONS
#define O_GP_PP 0
#define O_GP_OD 1
#define O_AF_PP 2
#define O_AF_OD 3

// STATE DEFINITIONS
#define LOW  0
#define HIGH 1


// GPIO INTINITIALISE 
void init_GP(unsigned short port, unsigned short pin, unsigned short dir, unsigned short opt);

// READ GPIO PIN
int R_GP(unsigned short port, unsigned short pin);

// WRITE GPIO PIN
void W_GP(unsigned short port, unsigned short pin, unsigned short state);

// TOGGLE GPIO PIN
void toggle_GP(unsigned short port, unsigned short pin);

