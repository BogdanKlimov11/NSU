#include "control.h"
#include "object.h"

void ButtonsControl(ConfButton buttons[], ScreenObject screenObject[], uint8_t countObj, uint8_t* currMovedObj){
//	if(PushButtListnr(&buttons[0])){
//		(*currMovedObj)++;
//		if (*currMovedObj >= countObj){
//			*currMovedObj = 0;
//		}
//	}
	if(PushButtListnr(&buttons[0])){
		(*currMovedObj)++;
		if (*currMovedObj >= countObj){
			*currMovedObj = 0;
		}
//		screenScene->brightness--;
//		if(screenScene->brightness == 0)
//			screenScene->brightness = 8; 
	}
	if(PushButtListnr(&buttons[1])){
		screenObject[*currMovedObj].posX++;
	}
	if(PushButtListnr(&buttons[2])){
		screenObject[*currMovedObj].posX--;
	}
	if(PushButtListnr(&buttons[3])){
		screenObject[*currMovedObj].posY++;
	}
	if(PushButtListnr(&buttons[4])){
		screenObject[*currMovedObj].posY--;
	}
}

void ChangeCountInterruptsWait(Multim* multim, ConfButton buttons[]){
	if(PushButtListnr(&buttons[0])){
		multim->countInterruptsWait = 10;
//		screenScene->brightness--;
//		if(screenScene->brightness == 0)
//			screenScene->brightness = 8; 
	}
	if(PushButtListnr(&buttons[1])){
		multim->countInterruptsWait += 1;
	}
	if(PushButtListnr(&buttons[2])){
		if(multim->countInterruptsWait > 0){
			multim->countInterruptsWait -= 1;
		}
	}
	if(PushButtListnr(&buttons[3])){
		multim->countInterruptsWait += 10;
	}
	if(PushButtListnr(&buttons[4])){
		if(multim->countInterruptsWait > 0){
			multim->countInterruptsWait -= 10;
		}
	}
}
