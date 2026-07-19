#pragma once

struct Dial {
    float val;
    float min;
    float max;
    float stp;
    char label[10];
    bool text_options = false;
    int count = 0;
    char **options = nullptr;
};

class Panel {
    int size;
    Dial* dials;
    const char* pname;
public:
    Panel(int s, Dial d[], const char* n);
    int   count() const;
    const char* name() const;
    const char* label(int d_num) const;
    float dial(int d_num) const;
    float change(int d_num, bool add);
    float dmin(int d) const;
    float dmax(int d) const;
    void set(int d_num, float v);
};