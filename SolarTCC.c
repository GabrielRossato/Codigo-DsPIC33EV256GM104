/*
 * File:   SolarTCC.c
 * Author: Gabriel Rossato Francisco
 */
// DSPIC33EV256GM104 Configuration Bit Settings
// 'C' source line config statements

// FSEC
#pragma config BWRP = OFF       // Boot Segment Write-Protect Bit (Boot Segment may be written)
#pragma config BSS = DISABLED   // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSS2 = OFF       // Boot Segment Control Bit (No Boot Segment)
#pragma config GWRP = OFF       // General Segment Write-Protect Bit (General Segment may be written)
#pragma config GSS = DISABLED   // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF       // Configuration Segment Write-Protect Bit (Configuration Segment may be written)
#pragma config CSS = DISABLED   // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = DISABLE// Alternate Interrupt Vector Table Disable Bit  (Disable Alternate Vector Table)

// FBSLIM
#pragma config BSLIM = 0x1FFF   // Boot Segment Code Flash Page Address Limit Bits (Boot Segment Flash Page Address Limit (0-0x1FFF))

// FOSCSEL
#pragma config FNOSC = FRC      // Initial oscillator Source Selection Bits (Internal Fast RC (FRC))
#pragma config IESO = ON        // Two Speed Oscillator Start-Up Bit (Start up device with FRC,then automatically switch to user selected oscillator source)

// FOSC
#pragma config POSCMD = HS      // Primary Oscillator Mode Select Bits (HS Crystal Oscillator mode)
#pragma config OSCIOFNC = OFF   // OSC2 Pin I/O Function Enable Bit (OSC2 is clock output)
#pragma config IOL1WAY = ON     // Peripheral Pin Select Configuration Bit (Allow Only One reconfiguration)
#pragma config FCKSM = CSECMD   // Clock Switching Mode Bits (Clock Switching is enabled,Fail-safe Clock Monitor is disabled)
#pragma config PLLKEN = ON      // PLL Lock Enable Bit (Clock switch to PLL source will wait until the PLL lock signal is valid)

// FWDT
#pragma config WDTPOST = PS32768// Watchdog Timer Postscaler Bits (1:32,768)
#pragma config WDTPRE = PR128   // Watchdog Timer Prescaler Bit (1:128)
#pragma config FWDTEN = OFF     // Watchdog Timer Enable Bits (WDT and SWDTEN Disabled)
#pragma config WINDIS = OFF     // Watchdog Timer Window Enable Bit (Watchdog timer in Non-Window Mode)
#pragma config WDTWIN = WIN25   // Watchdog Window Select Bits (WDT Window is 25% of WDT period)

// FPOR
#pragma config BOREN0 = ON      // Brown Out Reset Detection Bit (BOR is Enabled)

// FICD
#pragma config ICS = PGD2       // ICD Communication Channel Select Bits (Communicate on PGEC2 and PGED2)

// FDMTINTVL
#pragma config DMTIVTL = 0xFFFF // Lower 16 Bits of 32 Bit DMT Window Interval (Lower 16 bits of 32 bit DMT window interval (0-0xFFFF))

// FDMTINTVH
#pragma config DMTIVTH = 0xFFFF // Upper 16 Bits of 32 Bit DMT Window Interval (Upper 16 bits of 32 bit DMT window interval (0-0xFFFF))

// FDMTCNTL
#pragma config DMTCNTL = 0xFFFF // Lower 16 Bits of 32 Bit DMT Instruction Count Time-Out Value (Lower 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF))

// FDMTCNTH
#pragma config DMTCNTH = 0xFFFF // Upper 16 Bits of 32 Bit DMT Instruction Count Time-Out Value (Upper 16 bits of 32 bit DMT instruction count time-out value (0-0xFFFF))

// FDMT
#pragma config DMTEN = ENABLE   // Dead Man Timer Enable Bit (Dead Man Timer is Enabled and cannot be disabled by software)

// FDEVOPT
#pragma config PWMLOCK = OFF    // PWM Lock Disable Bit !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#pragma config ALTI2C1 = OFF    // Alternate I2C1 Pins Selection Bit (I2C1 mapped to SDA1/SCL1 pins)

// FALTREG
#pragma config CTXT1 = NONE     // Interrupt Priority Level (IPL) Selection Bits For Alternate Working Register Set 1 (Not Assigned)
#pragma config CTXT2 = NONE     // Interrupt Priority Level (IPL) Selection Bits For Alternate Working Register Set 2 (Not Assigned)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include "xc.h"
#include "math.h"

char i=0;
char ReceivedChar = 0;
char ReceiverBuffer[10];

union Data1A{
    char SendChar [4];
    float TensionPainel;
};

union Data1B{
    char SendChar [4];
    float ReferenciaVPainel;
};

union Data1C{
    char SendChar [4];
    float ControlVPainelOut;
};

union Data2{
    char SendChar [4];
    float CurrentPainel;
};

union Data3{
    char SendChar [4];
    float TensionLoad;
};


union Data4{
    char SendChar [4];
    float SensorX;
};

union Data5{
    char SendChar [4];
    float SensorY;
};

union Data6{
    char SendChar [4];
    float ControlX;
};

union Data7{
    char SendChar [4];
    float ControlY;
};

union Data1A data1A;
union Data1B data1B;
union Data1C data1C;
union Data2 data2;
union Data3 data3;

union Data4 data4;
union Data5 data5;
union Data6 data6;
union Data7 data7;


int EnableMPPT  = 0;
int EnableControlVpv  = 0;

double TensionPV = 0;
double TensionPV_1 = 0;
double CurrentPV = 0;
double CurrentPV_1 = 0;
double PowerPV = 0;
double PowerPV_1 = 0;
double TensionLoad = 0;

//PI conversor Buck
double ReferenciaVpv = 0;
double ErroVpv = 0;
double ErroVpv_1 = 0;
double IntegratorPI = 0;
double AntiWindup = 0;
double ControlPIoutput = 0;


int p=0;
int n=0;
int n_1=0;
int n_2=0;
double Sensor_LDR[5][3];
double FilterON_LDR [5][3];
double FilterOFF_LDR [5][3];

double SumLDRs = 0;
double XaxisError = 0;
double XaxisControl = 0;
double YaxisError = 0;
double YaxisControl = 0;

int enableFunctionTransfer=0; //Levantar FunçãoDeTransferencia

int enableRastreamentoDoSol=0; //Realiza o rastreamento do sol

int moveX=0;
int moveY=0;

int enableX=0;
int enableY=0;

int perpenXRight = 0;
int perpenXLeft = 0;
int perpenYRight = 0;
int penpenYLeft = 0;


void __attribute__((__interrupt__, no_auto_psv)) _AD1Interrupt(void) 
{
    
    AD1CON1bits.ASAM = 0; //Desabilita o AutoSAMP até que seja escrito 1 novamente
    
    
    //Adianta a comutação um tempo de amostragem
    //Porque eu leio a ADC, depois comuto o MUX
    switch((p+1)%5)
    {
        case 0: LATCbits.LATC1 = 0; //Bit0
                LATCbits.LATC0 = 0; //Bit1
                LATBbits.LATB3 = 0; //Bit2
                break;

        case 1: LATCbits.LATC1 = 1; //Bit0
                LATCbits.LATC0 = 0; //Bit1
                LATBbits.LATB3 = 0; //Bit2
                break;

        case 2: LATCbits.LATC1 = 0; //Bit0
                LATCbits.LATC0 = 1; //Bit1
                LATBbits.LATB3 = 0; //Bit2
                break;

        case 3: LATCbits.LATC1 = 1; //Bit0
                LATCbits.LATC0 = 1; //Bit1
                LATBbits.LATB3 = 0; //Bit2
                break;

        case 4: LATCbits.LATC1 = 0; //Bit0
                LATCbits.LATC0 = 0; //Bit1
                LATBbits.LATB3 = 1; //Bit2
                break;
            
    }
    
    
    IFS0bits.AD1IF = 0;   //Clear adc Flag interrupt

}


void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) //10kHz sample
{ 
    //LATBbits.LATB9 = i;
    //i=~i;     

    //LDR reading ADC mux comutate

    Sensor_LDR[p][n] = ADC1BUF4;
       
    //Filters IIR 2 order
    switch (n)
    {
        case 0: n_1 = 2;
                n_2 = 1; break;

        case 1: n_1 = 0;
                n_2 = 2; break;

        case 2: n_1 = 1;
                n_2 = 0; break;
    }
        
    
    //2 order IIR LowPass Chebyshev Filter 
    //fsample = 2kHz;
    
    //fc = 0.1Hz
    FilterON_LDR[p][n] = 0.00999692396850083*Sensor_LDR[p][n] - 0.0199936506062813*Sensor_LDR[p][n_1] + 0.00999692396850083*Sensor_LDR[p][n_2]
                         + 1.99937482849879*FilterON_LDR[p][n_1] - 0.999375025829507*FilterON_LDR[p][n_2];
    //fc = 10Hz
    FilterOFF_LDR[p][n] = 0.0101758827036283*Sensor_LDR[p][n] - 0.0184071801973730*Sensor_LDR[p][n_1] + 0.0101758827036283*Sensor_LDR[p][n_2]
                         + 1.93697498762654*FilterOFF_LDR[p][n_1] - 0.938919572836420*FilterOFF_LDR[p][n_2];

        
    //Commute LDRs mux for reading the sensors
    if(p>=4)
    {
        p=0;
        if(n>=2) n=0;
        else n = n+1;
    }
    else p = p+1;
  
    //switch(p)
    //{
    //  Relação das portaas com LDRs     
    //}
    
      
    //Current Tension Mesure
    TensionPV = ADC1BUF0*0.01465201465;  //5*12/4095 Resistive Divisor G=1/12
    CurrentPV = ADC1BUF1; //Deve ser feito assim. Não juntar com a de baixo
    //CurrentPV =(CurrentPV-2047)*0.0185000185;  //(ADC-2047)*(5*1000)/(66*4095) Current Sensor ACS712 30A (66 mV/A)
    CurrentPV = (CurrentPV - 2047)*0.0066000066; //(ADC-2047)*(5*1000)/(185*4095) Current Sensor ACS712 5A (185 mV/A)
    TensionLoad = ADC1BUF2*0.01465201465;    //5*12/4095 Resistive Divisor G=1/12 
    
    
    AD1CON1bits.ASAM = 1; //Habilita o AutoSAMP novamente para realizar a nova leitura das ADCs
    
    
    ////////Controle PI para controlar tensçao Vpv
    if(EnableControlVpv == 1)
    {
        //fs = 10kHz
        //Ki = 9.8012
        //Kp = 0.0893 
        //ReferenciaVpv = 20;
       
        //ErroVpv = (ReferenciaVpv - TensionPV);
        ErroVpv = (ReferenciaVpv - TensionPV) + AntiWindup/1000; //KantiWindUp = 1400*9.8012/1000
        
        IntegratorPI = IntegratorPI + (ErroVpv + ErroVpv_1)/(2*10000); //Trapezio 10kHz
        
        //ControlPIoutput = -(ErroVpv*0.0141 + IntegratorPI*5.8738)*1400 //Testado
        ControlPIoutput = -(ErroVpv*0.0893 + IntegratorPI*9.8012)*1400;
        

        if((ControlPIoutput>=0)&&(ControlPIoutput<=1400))
        {
            PDC1 = ControlPIoutput;
            AntiWindup = 0;
        }
        else
        {
            if(ControlPIoutput<0)
            {   
                PDC1 = 0;
                AntiWindup = ControlPIoutput;
            }
            
            if(ControlPIoutput>1400)
            {
                PDC1 = 1400;
                AntiWindup = ControlPIoutput - 1400;
            }
        }
        
        ErroVpv_1 = ErroVpv;
        
    }
    
    
    IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
}


void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) //500Hz periodic 
{ 
    
    //LATBbits.LATB9 = i;
    //i=~i;  
    
    /*
    //Obter a FunçãoDeTransferencia///////////////////////////////////////////
    if(enableFunctionTransfer==1)
    {
        SumLDRs = (Sensor_LDR[0][n] + Sensor_LDR[1][n] + Sensor_LDR[2][n] + Sensor_LDR[3][n] + Sensor_LDR[4][n])/5;
        
        //Eixo X
        //Degrau aplicado tempo 0 com amplitude = 30
        //Ganho da malha de realimentação = 60
        
        XaxisError = (Sensor_LDR[1][n] - Sensor_LDR[2][n])/SumLDRs; //Erro normalizado pela media de todos os sensores

        //XaxisControl = 5*XaxisError*65535; //Normaliza a saida de 0 a 1
        XaxisControl = (30-(60*XaxisError))*65535; //Normaliza a saida de 0 a 1

        if(XaxisControl>=0)
        {
            LATCbits.LATC5 = 1;
            LATCbits.LATC4 = 0;
        }
    
        if(XaxisControl<0)
        {
            LATCbits.LATC5 = 0;
            LATCbits.LATC4 = 1;
        }

        if((fabs(XaxisControl)>0)&&(fabs(XaxisControl)<65535)) PDC2 = fabs(XaxisControl);
        if(fabs(XaxisControl)<=0) PDC2 = 0;
        if(fabs(XaxisControl)>=65535) PDC2 = 65535;
        
        
        //Eixo Y
        //Degrau aplicado tempo 0 com amplitude = 30
        //Ganho da malha de realimentação = 40
        YaxisError = (Sensor_LDR[3][n] - Sensor_LDR[4][n])/SumLDRs; //Erro normalizado pela media de todos os sensores
    
        YaxisControl = (30-(40*YaxisError))*65535; //Normaliza a saida de 0 a 1

        if(YaxisControl>=0)
        {
            LATCbits.LATC3 = 1;
            LATAbits.LATA9 = 0;
        }

        if(YaxisControl<0)
        {
            LATCbits.LATC3 = 0;
            LATAbits.LATA9 = 1;
        }

        if((fabs(YaxisControl)>0)&&(fabs(YaxisControl)<65535)) PDC3 = fabs(YaxisControl);
        if(fabs(YaxisControl)<=0) PDC3 = 0;
        if(fabs(YaxisControl)>=65535) PDC3 = 65535;
        
    }*/
    
    
    //Rastrear Solar efeito girassol////////////////////////////////////////////
    //Controle Proporcional K
    
    //SumLDRs = (Sensor_LDR[0][n] + Sensor_LDR[1][n] + Sensor_LDR[2][n] + Sensor_LDR[3][n] + Sensor_LDR[4][n])/5;
    //XaxisError = (Sensor_LDR[1][n] - Sensor_LDR[2][n])/SumLDRs; //Erro normalizado pela media de todos os sensores
    //YaxisError = (Sensor_LDR[3][n] - Sensor_LDR[4][n])/SumLDRs; //Erro normalizado pela media de todos os sensores
    
    //enableRastreamentoDoSol = 1;
    
    if(enableRastreamentoDoSol == 1)
    {
        //Controle Proporcional K
        SumLDRs = (Sensor_LDR[0][n] + Sensor_LDR[1][n] + Sensor_LDR[2][n] + Sensor_LDR[3][n] + Sensor_LDR[4][n])/5;

        if(enableX==1)
        {
            //Axis X Control
            XaxisError = (Sensor_LDR[1][n] - Sensor_LDR[2][n])/SumLDRs; //Erro normalizado pela media de todos os sensores

            XaxisControl = -8.29*XaxisError*65535; //Normaliza a saida de 0 a 1

            if(XaxisControl>=0)
            {
                LATCbits.LATC5 = 1;
                LATCbits.LATC4 = 0;
            }

            if(XaxisControl<0)
            {
                LATCbits.LATC5 = 0;
                LATCbits.LATC4 = 1;
            }

            if((fabs(XaxisControl)>8000)&&(fabs(XaxisControl)<65535)) PDC2 = fabs(XaxisControl);
            if(fabs(XaxisControl)<=8000) PDC2 = 8000;
            if(fabs(XaxisControl)>=65535) PDC2 = 65535;
        }

        if(enableY==1)
        {
            //Axis Y Control
            YaxisError = (Sensor_LDR[3][n] - Sensor_LDR[4][n])/SumLDRs; //Erro normalizado pela media de todos os sensores

            YaxisControl = -8.03*YaxisError*65535; //Normaliza a saida de 0 a 1

            if(YaxisControl>=0)
            {
                LATCbits.LATC3 = 1;
                LATAbits.LATA9 = 0;
            }

            if(YaxisControl<0)
            {
                LATCbits.LATC3 = 0;
                LATAbits.LATA9 = 1;
            }

            if((fabs(YaxisControl)>4000)&&(fabs(YaxisControl)<65535)) PDC3 = fabs(YaxisControl);
            if(fabs(YaxisControl)<=4000) PDC3 = 4000;
            if(fabs(YaxisControl)>=65535) PDC3 = 65535;
        }
    
    }
    
    
    //Controle por histerese mesclado ao classico///////////////////////////////
 
    //Eixo X
    if((FilterON_LDR[1][n]<FilterON_LDR[0][n])&&(moveX==0))
    {
        //LATCbits.LATC5 = 1;
        //LATCbits.LATC4 = 0;
        //PDC2 = 65535;
        moveX = 1;
        enableX = 1;
    }
        
    if((FilterOFF_LDR[2][n]<FilterOFF_LDR[1][n])&&(moveX==1))
    {
        LATCbits.LATC5 = 0;
        LATCbits.LATC4 = 0;
        PDC2 = 0;
        moveX = 0;
        enableX = 0;
    }
        
    if((FilterON_LDR[2][n]<FilterON_LDR[0][n])&&(moveX==0))
    {
        //LATCbits.LATC5 = 0;
        //LATCbits.LATC4 = 1;
        //PDC2 = 65535;
        moveX = -1;
        enableX = 1;
    }
        
    if((FilterOFF_LDR[1][n]<FilterOFF_LDR[2][n])&&(moveX==-1))
    {
        LATCbits.LATC5 = 0;
        LATCbits.LATC4 = 0;
        PDC2 = 0;
        moveX = 0;
        enableX = 0;
    }
        
    //Eixo Y    
    if((FilterON_LDR[3][n]<FilterON_LDR[0][n])&&(moveY==0))
    {
        //LATCbits.LATC3 = 1;
        //LATAbits.LATA9 = 0;
        //PDC3 = 65535;
        moveY = 1;
        enableY = 1;
    }
        
    if((FilterOFF_LDR[4][n]<FilterOFF_LDR[3][n])&&(moveY==1))
    {
        LATCbits.LATC3 = 0;
        LATAbits.LATA9 = 0;
        PDC3 = 0;
        moveY = 0;
        enableY = 0;    
    }
        
    if((FilterON_LDR[4][n]<FilterON_LDR[0][n])&&(moveY==0))
    {
        //LATCbits.LATC3 = 0;
        //LATAbits.LATA9 = 1;
        //PDC3 = 65535;
        moveY = -1;
        enableY = 1;
    }
        
    if((FilterOFF_LDR[3][n]<FilterOFF_LDR[4][n])&&(moveY==-1))
    {
        LATCbits.LATC3 = 0;
        LATAbits.LATA9 = 0;
        PDC3 = 0;
        moveY = 0;
        enableY = 0;
    }
   
     
    //Perpendicular solo////////////////////////////////////////////////////////
    
    if(perpenXRight == 1)
    {
        switch (PORTBbits.RB4)
        {
            case 0:
                PDC2 = 65535;
                LATCbits.LATC5 = 1;
                LATCbits.LATC4 = 0;
                break;
            
            case 1:
                PDC2 = 0;
                LATCbits.LATC5 = 0;
                LATCbits.LATC4 = 0;
                perpenXRight = 0;
                break;
        }
    }    
    
    if(perpenXLeft == 1)
    {
        switch (PORTBbits.RB4)
        {
            case 0:
                PDC2 = 65535;
                LATCbits.LATC5 = 0;
                LATCbits.LATC4 = 1;
                break;
            
            case 1:
                PDC2 = 0;
                LATCbits.LATC5 = 0;
                LATCbits.LATC4 = 0;
                perpenXLeft = 0;
                break;
        }
        
    }
     
    if(perpenYRight == 1)
    {
        switch (PORTAbits.RA8)
        {
            case 0:
                PDC3 = 65535;
                LATCbits.LATC3 = 1;
                LATAbits.LATA9 = 0;
                break;
            
            case 1:
                PDC3 = 0;
                LATCbits.LATC3 = 0;
                LATAbits.LATA9 = 0;
                perpenYRight = 0;
                break;
        }
        
    }

    if(penpenYLeft == 1)
    {
        switch (PORTAbits.RA8)
        {
            case 0:
                PDC3 = 65535;
                LATCbits.LATC3 = 0;
                LATAbits.LATA9 = 1;
                break;
            
            case 1:
                PDC3 = 0;
                LATCbits.LATC3 = 0;
                LATAbits.LATA9 = 0;
                penpenYLeft = 0;
                break;
        }
    
    }

    
    //Rastreio Maximo Ponto Potencia MPPT///////////////////////////////////////
    
    if (EnableMPPT  == 1)
    {
        //LATBbits.LATB9 = 1;
        //DeltaVpv = 0.05V;
        
        PowerPV = TensionPV*CurrentPV;
        
        if(PowerPV - PowerPV_1 > 0)
        {
            
            if(TensionPV - TensionPV_1 > 0)
            {
                ReferenciaVpv = ReferenciaVpv + 0.05;
            }
            
            if(TensionPV - TensionPV_1 <= 0)
            {
                ReferenciaVpv = ReferenciaVpv - 0.05;
            }
            
        }
        
        if(PowerPV - PowerPV_1 < 0)
        {
            
            if(TensionPV - TensionPV_1 >= 0)
            {
                ReferenciaVpv = ReferenciaVpv - 0.05;
            }
            
            if(TensionPV - TensionPV_1 < 0)
            {
                ReferenciaVpv = ReferenciaVpv + 0.05;
            }
            
        }
        
        
        TensionPV_1 = TensionPV;
        CurrentPV_1 = CurrentPV;
        PowerPV_1 =  PowerPV;
        
        //Saturador entre 5V e 50V para P&O
        if(ReferenciaVpv < 5) ReferenciaVpv = 5;
        if(ReferenciaVpv > 50) ReferenciaVpv = 50;

    }
     
            
     IFS0bits.T2IF = 0; // Clear Timer2 Interrupt Flag
}


void __attribute__ ((__interrupt__,no_auto_psv)) _U1RXInterrupt(void)
{
    //U1TXREG = ReceivedChar;
    if(U1STAbits.OERR == 1) U1STAbits.OERR = 0;
    
    ReceivedChar = 0;       //Clear Buffer
    ReceivedChar = U1RXREG; //Receiver Char
    
    switch (ReceivedChar){
        
        //Envio variaveis internas do DsPic///////////////
        case '{':
            data1A.TensionPainel = TensionPV;
            for(i=0;i<4;i++) U1TXREG = data1A.SendChar[i];
            break;
        
        case '$':
            data1B.ReferenciaVPainel = ReferenciaVpv;
            for(i=0;i<4;i++) U1TXREG = data1B.SendChar[i];
            break;
        
        case '#':
            data1C.ControlVPainelOut = ControlPIoutput;
            for(i=0;i<4;i++) U1TXREG = data1C.SendChar[i];
            break;
        
        case '}':
            data2.CurrentPainel = CurrentPV;
            for(i=0;i<4;i++) U1TXREG = data2.SendChar[i];
            break;
        
        case '!':
            data3.TensionLoad = TensionLoad; 
            for(i=0;i<4;i++) U1TXREG = data3.SendChar[i];
            break;
        
        case 'a':
            data4.SensorX = XaxisError;
            for(i=0;i<4;i++) U1TXREG = data4.SendChar[i];
            break;
        
        case 'b':
            data5.SensorY = YaxisError;
            for(i=0;i<4;i++) U1TXREG = data5.SendChar[i];
            break;
        
        case 'c':
            data6.ControlX = XaxisControl;
            for(i=0;i<4;i++) U1TXREG = data6.SendChar[i];
            break;
        
        case 'd':
            data7.ControlY = YaxisControl;
            for(i=0;i<4;i++) U1TXREG = data7.SendChar[i];
            break;
        
        //Controle da ações do sistema///////////////
        case 'm':
            EnableMPPT = 1;
            break;
        
        case 'v':
            ReferenciaVpv = 10; //Condição Inicial
            EnableControlVpv = 1;
            break;
        
        case 'r':
            //LATBbits.LATB9 = 1;
            enableRastreamentoDoSol = 1;
            enableX=1;
            enableY=1;
            break;
            
        case 's':
            enableRastreamentoDoSol = 0;
            enableX=0;
            enableY=0;
            PDC2 = 0;
            PDC3 = 0;
            break;
            
        case 'p':
            //LATBbits.LATB9 = 1;
            perpenXRight = 1;
            perpenXLeft = 0;
            perpenYRight = 1;
            penpenYLeft = 0;
            break;
            
        case 't':
            perpenXRight = 0;
            perpenXLeft = 1;
            perpenYRight = 0;
            penpenYLeft = 1;
            break;
            
        case 'f':
            enableFunctionTransfer = 1;
            break;
        
            
    }
    
    //U1TXREG = ReceivedChar;
    
    //data1.TensionPainel=1.23456789;
    //data2.CurrentPainel=20.5432112345;
    //data3.TensionLoad=25.23456789;
    //data4.SensorX = 1.23456;
    //data5.SensorY = 2.34561;
    //data6.ErroX = 3.45612;
    //data7.ErroY = 4.56123;
        
    //data1.TensionPainel=FilterON_LDR[0][n];
    //data2.CurrentPainel=FilterON_LDR[1][n];
    //data3.TensionLoad=FilterON_LDR[2][n];
    //data4.SensorX = FilterON_LDR[3][n];
    //data5.SensorY = FilterON_LDR[4][n];
    
    /*
    data1A.TensionPainel = TensionPV;
    data1B.ReferenciaVPainel = ReferenciaVpv;
    data1C.ControlVPainelOut = ControlPIoutput;
    
    data2.CurrentPainel = CurrentPV;
    data3.TensionLoad = TensionLoad; 
   
    data4.SensorX = XaxisError;
    data5.SensorY = YaxisError;
    data6.ControlX = XaxisControl;
    data7.ControlY = YaxisControl;
    
    
    if(ReceivedChar=='{') for(i=0;i<4;i++) U1TXREG = data1A.SendChar[i];
    if(ReceivedChar=='$') for(i=0;i<4;i++) U1TXREG = data1B.SendChar[i];
    if(ReceivedChar=='#') for(i=0;i<4;i++) U1TXREG = data1C.SendChar[i];
    
    if(ReceivedChar=='}') for(i=0;i<4;i++) U1TXREG = data2.SendChar[i];
    if(ReceivedChar=='!') for(i=0;i<4;i++) U1TXREG = data3.SendChar[i];
    
    if(ReceivedChar=='a') for(i=0;i<4;i++) U1TXREG = data4.SendChar[i];
    if(ReceivedChar=='b') for(i=0;i<4;i++) U1TXREG = data5.SendChar[i];
    if(ReceivedChar=='c') for(i=0;i<4;i++) U1TXREG = data6.SendChar[i];
    if(ReceivedChar=='d') for(i=0;i<4;i++) U1TXREG = data7.SendChar[i];
    
  
    if(ReceivedChar=='m') EnableMPPT = 1; 
    
    if(ReceivedChar=='v') EnableControlVpv = 1; 
    
    
    if(ReceivedChar=='r')
    {
        //LATBbits.LATB9 = 1;
        enableX=1;
        enableY=1;
        
        //Gambi
        PDC2 = 20000;
        PDC3 = 15000;
        
    }
    
    if(ReceivedChar=='s')
    {
        //LATBbits.LATB9 = 1;
        enableX=0;
        enableY=0;
             
    }   
    
    if(ReceivedChar=='p')
    {
         perpenXRight = 1;
         perpenYRight = 1;
         
         //Gambi
         PDC2 = 65535;
         PDC3 = 65535;
    }
   
    if(ReceivedChar=='t')
    {
        perpenXLeft = 1;
        penpenYLeft = 1;
    }
    
    if(ReceivedChar=='f') //Levantar FunçãoDeTransferencia
    {
        enableFunctionTransfer = 1;
    }
    */
    
    IFS0bits.U1RXIF = 0;
}  
 
    
int main(void) {
    
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    
    CLKDIVbits.PLLPRE = 0;        // N1 = PLLPRE+2 = 2
    PLLFBD = 33;                  // M = PLLDIV + 2 =35
    CLKDIVbits.PLLPOST = 0;       // N2= 2 x (PLLPOST + 1) =2
   
                                //16M*35/2*2=140MHz

    // Initiate Clock Switch to Primary Oscillator with PLL (NOSC = 0b011)
     __builtin_write_OSCCONH(0x03);
     __builtin_write_OSCCONL(0x01);

    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b011);

    // Wait for PLL to lock
     while(OSCCONbits.LOCK != 1); 
     
     
    // Initialization Code Timer1 Counter Mode
     
    T1CONbits.TON = 0;      // Disable Timer
    T1CONbits.TCS = 0;      // Select internal instruction cycle clock (Fosc/2)
    T1CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR1 = 0x00;            // Clear timer register
    //PR1 = 1399;              // Load the period value T=PR1+1 freq= 50kHz
    PR1 = 6999;              // Load the period value T=PR1+1 freq= 10kHz
    IPC0bits.T1IP = 5;      // Set Timer1 Interrupt Priority Level
    IFS0bits.T1IF = 0;      // Clear Timer1 Interrupt Flag
    IEC0bits.T1IE = 1;      // Enable Timer1 interrupt
    T1CONbits.TON = 1;      // Start Timer
    
    
    // Initialization Code Timer2 Counter Mode
     
    T2CONbits.TON = 0;      // Disable Timer
    T2CONbits.TCS = 0;      // Select internal instruction cycle clock (Fosc/2)
    T2CONbits.TCKPS = 0b01; // Select 1:8 Prescaler
    //TMR2 = 0x00;            // Clear timer register
    //PR2 = 8749;              // Load the period value T=PR1 +1 freq= 1kHz
    PR2 = 17499;              // Load the period value T=PR1 +1 freq= 500Hz
    IPC1bits.T2IP = 6;      // Set Timer1 Interrupt Priority Level
    IFS0bits.T2IF = 0;      // Clear Timer1 Interrupt Flag
    IEC0bits.T2IE = 1;      // Enable Timer1 interrupt
    T2CONbits.TON = 1;      // Start Timer
     
    
    //Serial UART configure///////////////////////////////////////////////////////////////////
       
    TRISCbits.TRISC6 = 0;  //Configure RP54 Output
    TRISCbits.TRISC7 = 1;  //Configure RP55 Input  (For input only PPS functionality does not have priority over TRISx)
    ANSELCbits.ANSC7 = 0;  //The ANSELx record has a default value of 0xFFFF
    
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));   // Unlock Registers
    RPOR6bits.RP54R = 1;     //Pin RP54 to the UART1 TX output registe
    RPINR18bits.U1RXR = 55;  //Pin RP55 to the UART1 RX input register
    __builtin_write_OSCCONL(OSCCON | (1<<6));    // Lock Registers
    
    //U1MODEbits.STSEL = 0; // 1-Stop bit
    // U1MODEbits.PDSEL = 0; // No Parity, 8-Data bits
    //U1MODEbits.ABAUD = 0; // Auto-Baud disabled
    //U1MODEbits.BRGH = 0; // Standard-Speed mode
    
    U1MODE = 0;               //clear mode register
    U1STA = 0;                //clear status register
    
    //U1BRG = 18;               // (70M/230.4k/16)-1 BaudRate 230.4k
    //U1MODEbits.BRGH = 0;      // Standard-Speed mode  (16x baud clock, Standard mode)
    U1BRG = 18;               // (70M/921.6k/4)-1 BaudRate 921.6k
    U1MODEbits.BRGH = 1;      // High-Speed mode  (4x baud clock, High-Speed mode)
    
    U1MODEbits.WAKE = 1;      // Wake-up enabled
    IEC0bits.U1TXIE = 0;      // Disable UART TX interrupt
    IPC3bits.U1TXIP = 0;      //Set Send Interrupt Priority Level
    IEC0bits.U1RXIE = 1;      //Interrupt after one RX character is receive
    IFS0bits.U1RXIF = 0;      //clear the receive flag
    IPC2bits.U1RXIP = 7;      //Set Receivel Interrupt Priority Level
    U1MODEbits.UARTEN = 1;    // Enable UART
    U1STAbits.UTXEN = 1;      // Enable UART TX
        
    
     //Configure ADC////////////////////////////////////////////////////////////////////////////
    
    TRISAbits.TRISA0 = 1;  //Defined RA0 input analog AN0
    ANSELAbits.ANSA0 = 1;
    TRISAbits.TRISA1 = 1;  //Defined RA1 input analog AN1
    ANSELAbits.ANSA1 = 1;
    TRISBbits.TRISB0 = 1;  //Defined RB0 input analog AN2
    ANSELBbits.ANSB0 = 1;
    TRISBbits.TRISB1 = 1;  //Defined RB1 input analog AN3
    ANSELBbits.ANSB1 = 1;
    TRISBbits.TRISB2 = 1;  //Defined RB2 input analog AN4
    ANSELBbits.ANSB2 = 1;

    // Initialize ADC module 
    AD1CON1 = 0x04E0; // Enable 12-bit mode, auto-sample and auto-conversion. Sampling begins when SAMP bit is set
    AD1CON2 = 0x0410; // Sample 5 channels alternately using channel scanning
    //AD1CON3 = 0x030E; // Tad = 15/70M = 114.3ns. Sample time = 3*TAD. Conversion 274.5 ksps (14+3)/Tad
    AD1CON3 = 0x0307; // Tad = 8/70M = 114.3ns. Sample time = 3*TAD. Conversion 514.7 ksps (14+3)/Tad
    AD1CON4 = 0x0000;
    
    AD1CSSH = 0x0000;
    AD1CSSL = 0x001F; // Select AN0 to AN4  for input scan

   // Assign MUXA inputs 
    AD1CHS0bits.CH0SA = 0; // CH0SA bits ignored for CH0 +ve input selection
    AD1CHS0bits.CH0NA = 0; // Select VREF- for CH0 -ve input
    
    //ADC interrupt config
    IPC3bits.AD1IP = 6;   //Set priority ADC interrupt
    IFS0bits.AD1IF = 0;   //Clear adc Flag interrupt
    IEC0bits.AD1IE = 1;   //Enable ADC interrupt
    AD1CON1bits.ADON = 1; //Enable ADC module
    
   
    //Configure PWM////////////////////////////////////////////////////////////////////////////
    
    //PWM Buck scrycrono Complementary 
    PHASE1 = 1400;      //Fpwm=140M/1400 = 100kHz
    //PDC1 = 700;        //Set Duty Cycles
    PDC1 = 0;           //Set Duty Cycles
    //DTR1 = 28;          //Set Dead time 200ns;
    //ALTDTR1 = 28;       //Set Dead time 200ns;
    DTR1 = 14;          //Set Dead time 100ns;
    ALTDTR1 = 14;       //Set Dead time 100ns;
    IOCON1 = 0xC000;    //Set PWM Mode to Complementary ans active PWM1H, PWM1L pins
    PWMCON1 = 0x0200;   // Set Independent Time Bases, Edge-Aligned Mode and Independent Duty Cycles
    FCLCON1 = 0x0003;   //Disabel Faults Pins
    
    //PWM Motor control Independent 

    PHASE2 = 65535;  //Fpwm=140M/65535 = 2.14kHz
    PHASE3 = 65535;
    PDC2 = 0;  //Set Duty Cycles
    PDC3 = 0;
    DTR2 = DTR3 = 0;  //Set Dead Time Values
    ALTDTR2 = ALTDTR3 = 0;
    IOCON2 = IOCON3 = 0x8400; //Set PWM Mode to Redundant and PWMxL disable
    PWMCON2 = PWMCON3 = 0x0200; //Set Independent Time Bases, Edge-Aligned Mode and Independent Duty Cycles
    FCLCON2 = FCLCON3 = 0x0003; //Configure Faults

    PTCON2 = 0x0000;    //1:1 Prescaler
    PTCON = 0x8000;     // Enable PWM Module
    
    
    //Output pots configure
    
    TRISBbits.TRISB9 = 0; 
    LATBbits.LATB9 = 0;
    
    //Driver H Bridge X
    TRISCbits.TRISC5 = 0; 
    LATCbits.LATC5 = 0;
    TRISCbits.TRISC4 = 0; 
    LATCbits.LATC4 = 0;
    
    //Driver H Bridge Y
    TRISCbits.TRISC3 = 0; 
    LATCbits.LATC3 = 0;
    TRISAbits.TRISA9 = 0; 
    LATAbits.LATA9 = 0;
    
    //Analog Mux LDR bit select
    TRISCbits.TRISC1 = 0; //Bit0
    LATCbits.LATC1 = 0;
    TRISCbits.TRISC0 = 0; //Bit1
    LATCbits.LATC0 = 0;
    TRISBbits.TRISB3 = 0; //Bit2
    LATBbits.LATB3 = 0;
    //0 = 0b000 Center LDR
    //1 = 0b001 Axis X Right LDR
    //2 = 0b010 Axis X Left LDR
    //3 = 0b011 Axis Y Right LDR
    //4 = 0b100 Axis Y Left LDR
    
    //Center Magnect sensor input
    TRISBbits.TRISB4 = 1;  
    TRISAbits.TRISA8 = 1; 


    while(1)
    {
        //Infinite loop
        //LATBbits.LATB9 = 1;
    }
    
    return 0;
   
}