#include "control.h"
#include "object.h"

void ObjectMove(ConfButton buttons[], ScreenObject screenObject[], uint8_t countBtn, uint8_t countObj, uint8_t* currMovedObj){
	if(PushButtListnr(&buttons[0])){
		(*currMovedObj)++;
		if (*currMovedObj >= countObj){
			*currMovedObj = 0;
		}
	}
	for (uint8_t i = 0; i < countBtn; i++){
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
}
