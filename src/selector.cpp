#include "selector.h"
#include "robot-config.h"
#include "liblvgl/lvgl.h"
#include "utils/utils.h"
#include "routes.h"
#include "utils/timer.h"
#include "macros.h"
#include "gif-pros/gifclass.hpp"

int selectedIdx = 0;

enum DisplayMode {
    COLOR,
    AUTON,
    SELECTED,
    MIKU
};

Auton selectedAuton;

DisplayMode currentDisplay = COLOR;

lv_obj_t* red = nullptr;
lv_obj_t* blue = nullptr;
lv_obj_t* red_label = nullptr;
lv_obj_t* blue_label = nullptr;

lv_obj_t* redContainer = nullptr;
lv_obj_t* blueContainer = nullptr;

std::vector<lv_obj_t*> redList;
std::vector<lv_obj_t*> blueList;

lv_obj_t* box = nullptr;
lv_obj_t* label = nullptr;

lv_obj_t* countdown = nullptr;
lv_obj_t* countdownTime = nullptr;

Color selectedColor = NONE;

bool prevLeft = false;
bool prevRight = false;
bool newLeft = false;
bool newRight = false;
bool newBoth = false;

bool waitingForBothRelease = false;
bool bothDetected = false;

void updateSensors() {
    newLeft = false;
    newRight = false;
    newBoth = false;

    bool getLeft = false;
    bool getRight = false;

    if (leftDist.get() < 10) getLeft = true;
    if (rightDist.get() < 10) getRight = true;

    // Detect both hands down (only once)
    if (getLeft && getRight && !bothDetected) {
        waitingForBothRelease = true;
        bothDetected = true;
    }

    // Wait for both to be released after both were detected
    if (waitingForBothRelease && !getLeft && !getRight) {
        newBoth = true;
        waitingForBothRelease = false;
        bothDetected = false;
    }

    // Reset `bothDetected` if hands were not both down anymore
    if (!(getLeft && getRight)) {
        bothDetected = false;
    }

    // Handle individual taps only if not doing a both gesture
    if (!waitingForBothRelease) {
        if (!getLeft && prevLeft) newLeft = true;
        if (!getRight && prevRight) newRight = true;
    }

    prevLeft = getLeft;
    prevRight = getRight;
}

void createAutonList(Color color) {
    if(color == RED) {
        lv_obj_clear_flag(redContainer, LV_OBJ_FLAG_HIDDEN); // Show red container

        for(int i = 0; i < redAutons.size(); i++) {
            Auton auton = redAutons[i];
            lv_obj_t* btn = lv_btn_create(redContainer); // Attach to redContainer
            lv_obj_set_size(btn, SCREEN_WIDTH, 40);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x550000), LV_PART_MAIN);
            
            lv_obj_t* lbl = lv_label_create(btn);
            lv_label_set_text(lbl, auton.name.c_str());
            lv_obj_center(lbl);
            
            redList.push_back(btn);
        }
    } else if(color == BLUE) {
        lv_obj_clear_flag(blueContainer, LV_OBJ_FLAG_HIDDEN); // Show blue container

        for(int i = 0; i < blueAutons.size(); i++) {
            Auton auton = blueAutons[i];
            lv_obj_t* btn = lv_btn_create(blueContainer); // Attach to blueContainer
            lv_obj_set_size(btn, SCREEN_WIDTH, 40);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x000055), LV_PART_MAIN);
            
            lv_obj_t* lbl = lv_label_create(btn);
            lv_label_set_text(lbl, auton.name.c_str());
            lv_obj_center(lbl);
            
            blueList.push_back(btn);
        }
    }
    lv_obj_t* backBtn = lv_btn_create(color == RED ? redContainer : blueContainer);
    lv_obj_set_size(backBtn, SCREEN_WIDTH, 40);
    lv_obj_t* backLbl = lv_label_create(backBtn);
    lv_label_set_text(backLbl, "< Back to Color Select");
    lv_obj_center(backLbl);

    (color == RED ? redList : blueList).push_back(backBtn);
}

void clearAutonList(Color color) {
    std::vector<lv_obj_t*>& list = (color == RED ? redList : blueList);
    for (lv_obj_t* obj : list) {
        lv_obj_del(obj);
    }
    list.clear();

    if (color == RED) {
        lv_obj_add_flag(redContainer, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(blueContainer, LV_OBJ_FLAG_HIDDEN);
    }
}

void highlightSelected(std::vector<lv_obj_t*>& list) {
    for (int i = 0; i < list.size(); i++) {
        lv_obj_t* btn = list[i];
        if (i == selectedIdx) {
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x666666), LV_PART_MAIN); // gray highlight
        } else {
            lv_obj_set_style_bg_color(btn, lv_color_hex(i == list.size() - 1 ? 0xAAAAAA : 0x550000), LV_PART_MAIN); // Normal or grey for back option
        }
    }
}

void display() {

    Timer timer(5000);
    timer.pause();

    box = lv_obj_create(lv_scr_act());
    lv_obj_set_size(box, SCREEN_WIDTH, 50);
    lv_obj_set_pos(box, 0, 0);
    lv_obj_set_style_bg_color(box, lv_color_hex(0x000000), LV_PART_MAIN);

    label = lv_label_create(box);
    lv_label_set_text(label, "Select Color:");
    lv_obj_center(label);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_36, LV_PART_MAIN);

    red = lv_obj_create(lv_scr_act());
    lv_obj_set_size(red, HALF_WIDTH, SCREEN_HEIGHT - 50);
    lv_obj_set_pos(red, 0, 50);
    lv_obj_set_style_bg_color(red, lv_color_hex(0xFF0000), LV_PART_MAIN);

    blue = lv_obj_create(lv_scr_act());
    lv_obj_set_size(blue, HALF_WIDTH, SCREEN_HEIGHT - 50);
    lv_obj_set_pos(blue, HALF_WIDTH, 50);
    lv_obj_set_style_bg_color(blue, lv_color_hex(0x0000FF), LV_PART_MAIN);

    red_label = lv_label_create(red);
    lv_label_set_text(red_label, "Left - Red");
    lv_obj_center(red_label);
    lv_obj_set_style_text_font(red_label, &lv_font_montserrat_24, LV_PART_MAIN);

    blue_label = lv_label_create(blue);
    lv_label_set_text(blue_label, "Right - Blue");
    lv_obj_center(blue_label);
    lv_obj_set_style_text_font(blue_label, &lv_font_montserrat_24, LV_PART_MAIN);

    redContainer = lv_list_create(lv_scr_act());
    lv_obj_set_size(redContainer, SCREEN_WIDTH, SCREEN_HEIGHT - 50); 
    lv_obj_set_pos(redContainer, 0, 50);           
    lv_obj_set_flex_flow(redContainer, LV_FLEX_FLOW_COLUMN);

    blueContainer = lv_list_create(lv_scr_act());
    lv_obj_set_size(blueContainer, SCREEN_WIDTH, SCREEN_HEIGHT - 50); 
    lv_obj_set_pos(blueContainer, 0, 50);           
    lv_obj_set_flex_flow(blueContainer, LV_FLEX_FLOW_COLUMN);

    lv_obj_add_flag(redContainer, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(blueContainer, LV_OBJ_FLAG_HIDDEN);

    countdown = lv_obj_create(lv_scr_act());
    lv_obj_set_size(countdown, HALF_WIDTH, HALF_HEIGHT);
    lv_obj_center(countdown);
    lv_obj_set_style_bg_color(countdown, lv_color_hex(0x000000), LV_PART_MAIN);

    countdownTime = lv_label_create(countdown);
    lv_label_set_text(countdownTime, "");
    lv_obj_center(countdownTime);
    lv_obj_set_style_text_font(countdownTime, &lv_font_montserrat_48, LV_PART_MAIN);

    lv_obj_add_flag(countdown, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(countdownTime, LV_OBJ_FLAG_HIDDEN);

    while (true) {
        updateSensors();
        if(currentDisplay == COLOR) {
            if(newLeft) {
                if(!pros::competition::is_field_control()) master.rumble(".");
                lv_obj_add_flag(red, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(blue, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(red_label, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(blue_label, LV_OBJ_FLAG_HIDDEN);
                
                lv_label_set_text(label, "Red Autons");

                currentDisplay = AUTON; 
                selectedColor = RED;
                sortColor = BLUE;

                createAutonList(RED);
            } else if(newRight) {
                lv_obj_add_flag(red, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(blue, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(red_label, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(blue_label, LV_OBJ_FLAG_HIDDEN);

                lv_label_set_text(label, "Blue Autons");

                currentDisplay = AUTON;
                selectedColor = BLUE;
                sortColor = RED;

                createAutonList(BLUE);
            }
        }
        if(currentDisplay == AUTON) {
            std::vector<lv_obj_t*>& list = (selectedColor == RED ? redList : blueList);

            if (newBoth) {
                newBoth = false;
                if (selectedIdx == list.size() - 1) {
                    // Last option → go back to color selection
                    lv_label_set_text(label, "Select Color:");
                    lv_obj_clear_flag(red, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(blue, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(red_label, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(blue_label, LV_OBJ_FLAG_HIDDEN);
                    clearAutonList(RED);
                    clearAutonList(BLUE);
                    currentDisplay = COLOR;
                    selectedIdx = 0;
                } else {
                    currentDisplay = SELECTED;
                    if (selectedColor == RED) {
                        lv_obj_add_flag(redContainer, LV_OBJ_FLAG_HIDDEN);
                    } else {
                        lv_obj_add_flag(blueContainer, LV_OBJ_FLAG_HIDDEN);
                    }
                    std::string selectedName = (selectedColor == RED ? redAutons[selectedIdx].name : blueAutons[selectedIdx].name);
                    lv_label_set_text(label, ("Selected Auton: " + selectedName).c_str());
                    lv_obj_clear_flag(countdown, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_clear_flag(countdownTime, LV_OBJ_FLAG_HIDDEN);
                    timer.resume();
                }
            } else if (newLeft) {
                selectedIdx--;
                if (selectedIdx < 0) selectedIdx = list.size() - 1;
                highlightSelected(list);
                newLeft = false;
            } else if (newRight) {
                selectedIdx++;
                if (selectedIdx >= list.size()) selectedIdx = 0;
                highlightSelected(list);
                newRight = false;
            } 
        }
        if(currentDisplay == SELECTED) {
            if(timer.isDone()) {
                lv_obj_add_flag(countdown, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(countdownTime, LV_OBJ_FLAG_HIDDEN);
                if(selectedColor == RED) {
                    selectedAuton = redAutons[selectedIdx];
                } else {
                    selectedAuton = blueAutons[selectedIdx];
                }
                static Gif gif("/usd/miku.gif", lv_scr_act());
                break;
            }
            lv_label_set_text(countdownTime, std::to_string(timer.getTimeLeft() / 1000).c_str());
        }
        pros::delay(50);
    }
}

void initSelector() {

    pros::Task brainTask(display);
}