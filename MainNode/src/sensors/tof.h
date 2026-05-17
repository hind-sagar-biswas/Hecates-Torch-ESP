#pragma once

void tofInit();
float tofGetDistance();   // meters, -1.0 if no reading
bool tofForwardBlocked(); // true if dist < D_MIN or no reading