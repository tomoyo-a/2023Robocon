#ifndef _PPS_H
#define _PPS_H
#define SETX 0
#define SETY 1
#define SETA 3
void TalkToPps(int fd);
void Delay_ms(int time);
void WaitPpsReady(int fd);
void CorrectPps(int fd, char type, float setValue);
#endif