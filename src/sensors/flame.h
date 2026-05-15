#pragma once

void flameInit();
float flameGetNorm();   // analog → flame_norm [0,1]
bool flameGetDigital(); // digital pin → hard blocked flag