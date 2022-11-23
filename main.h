/* 
 * File:   main.h
 * Author: Ben Abbott
 *
 * Created on November 22, 2022, 12:09 PM
 */

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    int main();
    void setup(void);
    void setupTMR(void);
    void setupSPI(void);
    void delay_ms(uint32_t ms);
    void delay_us(uint32_t us);


#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

