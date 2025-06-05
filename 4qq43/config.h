#undef TAPPING_TERM
#define TAPPING_TERM 250

#define USB_SUSPEND_WAKEUP_DELAY 0
#define LAYER_STATE_8BIT
#define COMBO_COUNT 7
#define HCS(report) host_consumer_send(record->event.pressed ? report : 0); return false

#define RGB_MATRIX_STARTUP_SPD 60

