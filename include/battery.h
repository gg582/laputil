typedef struct __prim_battery_t {
    int remaining; 
    int current_avg;
    int power_avg;
    _Bool active;
} battery_t;
