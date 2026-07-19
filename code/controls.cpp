#include "controls.h"

Panel::Panel(int s, Dial d[], const char* n)
    : size(s), dials(d), pname(n) {}

int Panel::count() const           { return size; }
const char* Panel::name() const    { return pname; }
const char* Panel::label(int d) const { return dials[d].label; }
float Panel::dial(int d) const     { return dials[d].val; }

float Panel::change(int d, bool add)
{
    float v = dials[d].val + (add ? dials[d].stp : -dials[d].stp);
    if (v > dials[d].max) v = dials[d].max;   /* clamp instead of running away */
    if (v < dials[d].min) v = dials[d].min;
    dials[d].val = v;
    return v;
}

float Panel::dmin(int d) const { return dials[d].min; } //used in lcd drawing
float Panel::dmax(int d) const { return dials[d].max; }

void Panel::set(int d, float v){
	if (!(v == v)) v = dials[d].min;
	if (v > dials[d].max) v = dials[d].max;
	if (v < dials[d].min) v = dials[d].min;
	dials[d].val = v;
}