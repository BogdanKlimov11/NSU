#include "button.h"

ConfButton InitButton(GroundBtnRead groundBtnRead){
	ConfButton button;
	button.pushBtn = 0;
	button.releaseBtn = 0;
	button.listenRange = 10;
	button.countWithoutChange = 0;
	button.currStableState = 0;
	button.currInstantState = 0;
	button.prevStableState = 0;
	button.prevInstantState = 0;
	button.groundBtnRead = groundBtnRead;
	return button;
}

//uint8_t ButtonReadCurrInstantState(){
  //  return (GPIO_IDR_0 & GPIOA->IDR) ? 1 : 0;
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
					if ((button->currStableState == 1) && (button->prevStableState == 0)){
							button->pushBtn = 1;
						}
						if ((button->currStableState == 0) && (button->prevStableState == 1)){
							button->releaseBtn = 1;
						}
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
		if (button->pushBtn == 1){
			button->pushBtn = 0;
			return 1;
		}
	}
	return 0;
}

uint8_t ReleasingButtListnr(ConfButton* button){
	if (button!=NULL){
		if (button->releaseBtn == 1){
			button->releaseBtn = 0;
			return 1;
		}
	}
	return 0;
}

void Handler5Button(ConfButton buttons[]){
	HandlerButton(&buttons[0]);
	
	GPIOA->ODR |= GPIO_ODR_15;
	HandlerButton(&buttons[1]);
	HandlerButton(&buttons[2]);
	GPIOA->ODR &= ~GPIO_ODR_15;
		
	GPIOC->ODR |= GPIO_ODR_12;
	HandlerButton(&buttons[3]);
	HandlerButton(&buttons[4]);
	GPIOC->ODR &= ~GPIO_ODR_12;
}
