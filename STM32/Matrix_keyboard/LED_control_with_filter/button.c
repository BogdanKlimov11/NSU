#include "button.h"

ConfButton InitButton(GroundBtnRead groundBtnRead){
	ConfButton button;
	button.listenRange = 5;
	button.countWithoutChange = 0;
	button.currStableState = 0;
	button.currInstantState = 0;
	button.prevStableState = 0;
	button.prevInstantState = 0;
	button.groundBtnRead = groundBtnRead;
	return button;
}


//uint8_t ButtonReadCurrInstantState(){
//	return (GPIO_IDR_0 & GPIOA->IDR) ? 1 : 0;
//}


void HandlerButton(ConfButton* button){
	if (button!=NULL){
		button->prevStableState = button->currStableState;
		button->currInstantState = button->groundBtnRead();
		
		if (button->prevInstantState != button->currInstantState){
			button->countWithoutChange = 0;
		}
		else{
			if (button->countWithoutChange >= button->listenRange){
				button->prevStableState = button->currStableState;
				button->currStableState = button->currInstantState;
			}
			else{
				button->countWithoutChange++;
			}  
		}
		button->prevInstantState = button->currInstantState;
	}
}


uint8_t CurrStableState(ConfButton* button){
	if (button!=NULL){
    return button->currStableState;
	}
	return 0;
}

uint8_t PrevStableState(ConfButton* button){
	if (button!=NULL){
    return button->prevStableState;
	}
	return 0;
}


uint8_t PushButtListnr(ConfButton* button){
	if (button!=NULL){
    return ((button->currStableState == 1) && (button->prevStableState == 0)) ? 1 : 0;
	}
	return 0;
}


uint8_t ReleasingButtListnr(ConfButton* button){
	if (button!=NULL){
    return ((button->currStableState == 0) && (button->prevStableState == 1)) ? 1 : 0;
	}
	return 0;
}
